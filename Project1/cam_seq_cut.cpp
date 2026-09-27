#include "cam_seq_cut.hpp"

#include <trueform/io.hpp>
#include <trueform/csg.hpp>
#include <trueform/core.hpp>

#include <algorithm>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>

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
    auto shape = tf::read_stl(shape_path);
    auto tool = tf::read_stl(tool_path);

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

    for (float x = x_min; x <= x_max; x += step)
    {
        auto tx = tf::make_transformation_from_translation(
            tf::vector<float, 3>{ x, 0.0f, 0.0f });
        auto tool_transformed = tool.polygons() | tf::tag(tx);

        auto [next, labels, face_labels] = tf::make_boolean(
            current.polygons(),
            tool_transformed,
            tf::boolean_op::left_difference);

        current = std::move(next);
        ++count;

        if (count % 10 == 0)
            std::cout << "  step " << count << " (x=" << x << "): "
                      << current.faces().size() << " faces\n";
    }

    std::cout << "cam_seq_cut done. total steps=" << count
              << ", final faces=" << current.faces().size() << "\n";
    return current;
}

tf::polygons_buffer<int, float, 3, 3> cam_seq_cut2(tf::polygons_buffer<int, float, 3, 3>& model, tf::polygons_buffer<int, float, 3, 3>& tool, float step,
	float x_min,
	float x_max)
{
	if (model.faces().size() == 0)
		throw std::runtime_error("model mesh is empty");
	if (tool.faces().size() == 0)
		throw std::runtime_error("tool mesh is empty");

	// 若范围未指定，则自动根据包围盒计算，使 tool 沿 X 轴完整扫过 model
	if (x_max <= x_min)
	{
		auto ma = compute_aabb(model);
		auto ta = compute_aabb(tool);
		float tool_half_x = (ta.x1 - ta.x0) * 0.5f;
		x_min = ma.x0 - tool_half_x;
		x_max = ma.x1 + tool_half_x;
	}

	std::cout << "cam_seq_cut2: step=" << step
	          << ", x_range=[" << x_min << ", " << x_max << "]\n";
	std::cout << "  model: " << model.faces().size() << " faces\n";
	std::cout << "  tool:  " << tool.faces().size() << " faces\n";

	// 复制一份 model，避免破坏调用方传入的网格
	polygons_buffer_t current = model;
	int count = 0;

	// tool 沿 X 轴正方向以 step 为步长平移，每步做布尔差集 current - tool_at_x
	for (float x = x_min; x <= x_max; x += step)
	{
		auto tx = tf::make_transformation_from_translation(
			tf::vector<float, 3>{ x, 0.0f, 0.0f });
		auto tool_transformed = tool.polygons() | tf::tag(tx);

		auto [next, labels, face_labels] = tf::make_boolean(
			current.polygons(),
			tool_transformed,
			tf::boolean_op::left_difference);

		current = std::move(next);
		++count;

		if (count % 10 == 0)
			std::cout << "  step " << count << " (x=" << x << "): "
			          << current.faces().size() << " faces\n";
	}

	std::cout << "cam_seq_cut2 done. total steps=" << count
	          << ", final faces=" << current.faces().size() << "\n";
	return current;
}
