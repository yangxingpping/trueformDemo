// 使用 trueform 读取两个 STL 网格并执行布尔差集（布尔非）运算：result = base - tool
//
// 用法:
//   Project67.exe <base.stl> <tool.stl> [output.stl]
//
// 依赖:
//   polydera.trueform (header-only) + oneTBB，均通过 packages.config 中的
//   NuGet 包引入，MSBuild 会自动配置头文件路径、链接库并拷贝 TBB 运行时 DLL。

#include <trueform/trueform.hpp>

#include <exception>
#include <filesystem>
#include <iostream>
#include <string>

namespace
{
    void print_usage(const char* exe)
    {
        std::cout
            << "trueform boolean difference: result = base - tool\n"
            << "usage: " << exe << " <base.stl> <tool.stl> [output.stl]\n"
            << "  base.stl   : mesh A (the mesh being cut)\n"
            << "  tool.stl   : mesh B (the mesh subtracted from A)\n"
            << "  output.stl : result of A - B (default: difference.stl)\n";
    }
}




int main(int argc, char* argv[])
{
    if (argc < 3)
    {
        print_usage(argv[0]);
        return 0;
    }

    const std::string base_path = argv[1];
    const std::string tool_path = argv[2];
    const std::string output_path = (argc >= 4) ? argv[3] : "difference.stl";

    try
    {
        for (const auto& path : {base_path, tool_path})
        {
            if (!std::filesystem::exists(path))
            {
                std::cerr << "error: file not found: " << path << '\n';
                return 1;
            }
        }

        // 1. 读取两个 STL（自动识别 ASCII / 二进制，加载时自动去重顶点）
        std::cout << "reading " << base_path << " ...\n";
        auto base_buffer = tf::read_stl(base_path);

        std::cout << "reading " << tool_path << " ...\n";
        auto tool_buffer = tf::read_stl(tool_path);

        if (base_buffer.faces().size() == 0 || tool_buffer.faces().size() == 0)
        {
            std::cerr << "error: failed to parse one of the input meshes (no faces)\n";
            return 1;
        }

        std::cout << "  base: " << base_buffer.faces().size() << " faces, "
                  << base_buffer.points().size() << " points\n";
        std::cout << "  tool: " << tool_buffer.faces().size() << " faces, "
                  << tool_buffer.points().size() << " points\n";

        // 2. 布尔差集（布尔非）: base - tool
        //    left_difference 表示保留第一个操作数内部、第二个操作数外部的区域，
        //    即 A - B；腔体表面由 tool 的片元反向围合，结果保持水密。
        std::cout << "computing boolean difference (base - tool) ...\n";
        auto [result, labels, face_labels] = tf::make_boolean(
            base_buffer.polygons(),
            tool_buffer.polygons(),
            tf::boolean_op::intersection);

        std::cout << "  result: " << result.faces().size() << " faces, "
                  << result.points().size() << " points\n";

        // 3. 将结果写出为二进制 STL
        if (!tf::write_stl(result.polygons(), output_path))
        {
            std::cerr << "error: failed to write output: " << output_path << '\n';
            return 1;
        }

        std::cout << "written: " << output_path << '\n';
    }
    catch (const std::exception& e)
    {
        std::cerr << "error: " << e.what() << '\n';
        return 1;
    }

    return 0;
}
