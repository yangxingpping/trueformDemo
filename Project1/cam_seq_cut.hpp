#pragma once
#include <trueform/core/polygons_buffer.hpp>
#include <string>

using polygons_buffer_t = tf::polygons_buffer<int, float, 3, 3>;

// 沿 X 轴以 step 为步长对 shape 连续执行布尔差集（shape - tool_at_x）。
// x_min / x_max 为 tool 在 X 轴上的扫掠范围；若 x_max <= x_min，则自动根据
// shape 和 tool 的包围盒计算，使 tool 能完整覆盖 shape 的 X 范围。
polygons_buffer_t cam_seq_cut(
    const std::string& shape_path,
    const std::string& tool_path,
    float step,
    float x_min,
    float x_max);

tf::polygons_buffer<int, float, 3, 3> cam_seq_cut2(tf::polygons_buffer<int, float, 3, 3>& model, tf::polygons_buffer<int, float, 3, 3>& tool, float step,
	float x_min,
	float x_max);

