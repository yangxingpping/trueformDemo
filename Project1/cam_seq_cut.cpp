#include "cam_seq_cut.hpp"

#include <trueform/io.hpp>
#include <trueform/csg.hpp>
#include <trueform/core.hpp>
#include <trueform/remesh.hpp>
#include <trueform/clean.hpp>
#include <trueform/topology.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace
{
    struct Aabb
    {
        float x0, x1, y0, y1, z0, z1;
    };

    Aabb compute_aabb(const polygons_buffer_t& buf)
    {
        Aabb a{ std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(),
                std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(),
                std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity() };
        auto pts = buf.points();
        for (auto it = pts.begin(); it != pts.end(); ++it)
        {
            const float* p = (*it).data();
            a.x0 = std::min(a.x0, p[0]);
            a.x1 = std::max(a.x1, p[0]);
            a.y0 = std::min(a.y0, p[1]);
            a.y1 = std::max(a.y1, p[1]);
            a.z0 = std::min(a.z0, p[2]);
            a.z1 = std::max(a.z1, p[2]);
        }
        return a;
    }
}

polygons_buffer_t cam_seq_cut(
    const std::string& shape_path,
    const std::string& tool_path,
    float step,
    float x_min,
    float x_max)
{
    auto shape = tf::read_stl<int64_t>(shape_path);
    auto tool = tf::read_stl<int64_t>(tool_path);

    if (shape.faces().size() == 0)
        throw std::runtime_error("shape mesh is empty: " + shape_path);
    if (tool.faces().size() == 0)
        throw std::runtime_error("tool mesh is empty: " + tool_path);

    // 若范围未指定，则自动计算：让 tool 的 X 范围能覆盖 shape 的 X 范围
    if (x_max <= x_min)
    {
        auto sa = compute_aabb(shape);
        auto ta = compute_aabb(tool);
        float tool_half_x = (ta.x1 - ta.x0) * 0.5f;
        x_min = sa.x0 - tool_half_x;
        x_max = sa.x1 + tool_half_x;
    }

    std::cout << "cam_seq_cut: step=" << step
              << ", x_range=[" << x_min << ", " << x_max << "]\n";
    std::cout << "  shape: " << shape.faces().size() << " faces\n";
    std::cout << "  tool:  " << tool.faces().size() << " faces\n";

    polygons_buffer_t current = std::move(shape);
    int count = 0;

    for (int j = 0; j < 100; ++j)
    {
        const auto t_start = std::chrono::steady_clock::now();
        for (float x = x_min; x <= x_max; x += step)
        {
            auto tx = tf::make_transformation_from_translation(
                tf::vector<float, 3>{ x, 0.08*j, 0.0f });
            auto tool_transformed = tool.polygons() | tf::tag(tx);

            auto [next, labels, face_labels] = tf::make_boolean(
                current.polygons(),
                tool_transformed,
                tf::boolean_op::left_difference);

            current = std::move(next);
            ++count;

            if (count % 2 == 0) {
                /*std::cout << "  step " << count << " (x=" << x << "): "
                    << current.faces().size() << " faces\n";*/
                
				current = tf::cleaned(current.polygons());
            }

        }
		const auto t_end = std::chrono::steady_clock::now();

		auto ms = [](auto a, auto b)
			{
				return std::chrono::duration_cast<std::chrono::milliseconds>(b - a).count();
			};
		SPDLOG_INFO("total cost time={} ms", ms(t_start, t_end));
    }

    std::cout << "cam_seq_cut done. total steps=" << count
              << ", final faces=" << current.faces().size() << "\n";
    return current;
}

// cam_seq_cut2：高性能扫掠切削。
//
// 不再做 N 次链式布尔差集（每次都要对整个模型重建加速结构、重排布），
// 而是使用 trueform 的 N-form CSG 模块：
//   1. 把「模型（恒等变换）」和「沿 X 轴所有步进的刀具位姿」作为
//      N+1 个 form，一次性投入 tf::make_csg_graph —— 全部几何体在
//      单次 arrangement（全局拓扑）中完成求交与分类；
//   2. 用布尔表达式 difference(0, any_of(1..N)) 一次性求值
//      「模型 - 刀具扫掠体（所有刀具位姿的并集）」；
//   3. tf::make_csg_mesh 直接从全局拓扑中抽取结果网格。
// 模型只被切分一次，刀具位姿之间仅在真正重叠处求交，且全程由 TBB 并行。
tf::polygons_buffer<int64_t, float, 3, 3> cam_seq_cut2(tf::polygons_buffer<int64_t, float, 3, 3>& model, tf::polygons_buffer<int64_t, float, 3, 3>& tool, float step,
	float x_min,
	float x_max)
{
	if (model.faces().size() == 0)
		throw std::runtime_error("model mesh is empty");
	if (tool.faces().size() == 0)
		throw std::runtime_error("tool mesh is empty");
	if (!(step > 0.0f))
		throw std::runtime_error("step must be positive");

	// 若范围未指定，则自动根据包围盒计算，使 tool 沿 X 轴完整扫过 model
	if (x_max <= x_min)
	{
		auto ma = compute_aabb(model);
		auto ta = compute_aabb(tool);
		float tool_half_x = (ta.x1 - ta.x0) * 0.5f;
		x_min = ma.x0 - tool_half_x;
		x_max = ma.x1 + tool_half_x;
	}

	// 用整数步进避免浮点累加漂移；tool 沿 X 轴正方向逐位姿平移
	const int n_steps =
	    static_cast<int>(std::floor((x_max - x_min) / step + 1e-6f)) + 1;

	const auto t_start = std::chrono::steady_clock::now();
	std::cout << "cam_seq_cut2: step=" << step
	          << ", x_range=[" << x_min << ", " << x_max << "], steps=" << n_steps << "\n";
	std::cout << "  model: " << model.faces().size() << " faces\n";
	std::cout << "  tool:  " << tool.faces().size() << " faces\n";

	// ---- 1. 组装 forms：模型（恒等平移）+ 每个步进位姿的刀具 ----
	// 恒等平移只是为了让模型与刀具的 tagged 视图类型一致，可放进同一容器
	const auto id_tx = tf::make_transformation_from_translation(
	    tf::vector<float, 3>{ 0.0f, 0.0f, 0.0f });
	using form_t = decltype(model.polygons() | tf::tag(id_tx));

	std::vector<form_t> forms;
	

    int count{ 130 };

    tf::polygons_buffer<int64_t, float, 3, 3> result;
    tf::polygons_buffer<int64_t, float, 3, 3> cursrc = model;

    for (int k = 0; k < 2; ++k)
    {
        for (int j = 0; j < count; ++j)
        {

            forms.clear();
            forms.reserve((static_cast<std::size_t>(n_steps) + 1) * 3);
            forms.push_back(cursrc.polygons() | tf::tag(id_tx));
            for (int i = 0; i < n_steps; ++i)
            {
                float x = x_min + static_cast<float>(i) * step;
                forms.push_back(tool.polygons() | tf::tag(
                    tf::make_transformation_from_translation(tf::vector<float, 3>{ x, j * 0.08f, (-0.08)*k })));
            }
            const auto t_forms = std::chrono::steady_clock::now();

            // ---- 2. 一次性构建全局拓扑（arrangement + 分类）----
            // make_csg_graph 内部会对容器再做一次 make_range，且策略按原模板
            // 参数存储，因此这里必须传 tf::range 而不是 std::vector
            auto graph = tf::make_csg_graph(tf::make_range(forms));
            const auto t_graph = std::chrono::steady_clock::now();

            // ---- 3. 求值 result = model - union(tool_1..tool_N)，一次抽取结果 ----
            const int n_tools = static_cast<int>(forms.size()) - 1;
            auto sweep = tf::csg::any_of(tf::make_sequence_range(1, n_tools + 1));
            cursrc = tf::make_csg_mesh(graph, tf::csg::difference(0, sweep));
           
        }
    }


	
    result = cursrc;


	const auto t_end = std::chrono::steady_clock::now();

	auto ms = [](auto a, auto b)
	{
		return std::chrono::duration_cast<std::chrono::milliseconds>(b - a).count();
	};
    SPDLOG_INFO("total cost time={} ms", ms(t_start, t_end));
	return result;
}
