#include "rlc_executive/bt/tree_runner.hpp"

#include <behaviortree_cpp/basic_types.h>

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "rlc_executive/core/blackboard_keys.hpp"
#include "rlc_executive/core/runtime_context.hpp"

namespace rlc_executive
{
namespace
{

bool looksLikeSharedLibraryPath(const std::string& s)
{
  return s.size() >= 3 && (s.rfind(".so") == s.size() - 3);
}

std::vector<std::string> loadLibraryListFile(const std::string& path)
{
  std::vector<std::string> libs;

  std::ifstream in(path);
  if (!in.is_open())
  {
    throw std::runtime_error("TreeRunner: failed to open plugin list file: " + path);
  }

  std::string line;
  while (std::getline(in, line))
  {
    while (!line.empty() &&
           (line.back() == ' ' || line.back() == '\t' || line.back() == '\r'))
    {
      line.pop_back();
    }

    std::size_t i = 0;
    while (i < line.size() && (line[i] == ' ' || line[i] == '\t'))
    {
      ++i;
    }
    line = line.substr(i);

    if (line.empty() || line[0] == '#')
    {
      continue;
    }

    libs.push_back(line);
  }

  return libs;
}

}  // namespace

void TreeRunner::configure(const std::string& plugins_path_or_list_file)
{
  last_plugins_path_or_list_file_ = plugins_path_or_list_file;

  std::vector<std::string> libs;

  if (looksLikeSharedLibraryPath(plugins_path_or_list_file))
  {
    libs.push_back(plugins_path_or_list_file);
  }
  else
  {
    libs = loadLibraryListFile(plugins_path_or_list_file);
  }

  const std::filesystem::path list_file_dir =
      std::filesystem::path(plugins_path_or_list_file).parent_path();
  const std::filesystem::path prefix_dir =
      list_file_dir.parent_path().parent_path().parent_path();

  for (const auto& lib : libs)
  {
    const std::filesystem::path lib_path(lib);
    if (lib_path.is_absolute())
    {
      factory_.registerFromPlugin(lib_path.string());
    }
    else if (lib_path.has_parent_path())
    {
      factory_.registerFromPlugin((list_file_dir / lib_path).string());
    }
    else
    {
      factory_.registerFromPlugin((prefix_dir / "lib" / lib_path).string());
    }
  }
}

void TreeRunner::loadTreeFromXmlFile(const std::string& tree_xml_path,
                                     const std::shared_ptr<RuntimeContext>& ctx)
{
  last_tree_xml_path_ = tree_xml_path;

  blackboard_ = BT::Blackboard::create();
  blackboard_->set<std::shared_ptr<RuntimeContext>>(bb::RUNTIME_CONTEXT, ctx);

  tree_ = factory_.createTreeFromFile(tree_xml_path, blackboard_);
}

BT::NodeStatus TreeRunner::tick()
{
  switch (tick_option_)
  {
    case TickOption::ONCE_UNLESS_WOKEN_UP:
      return tickOnce();
    case TickOption::WHILE_RUNNING:
      return tickWhileRunning(tick_while_running_max_block_);
    case TickOption::EXACTLY_ONCE:
      return tickExactlyOnce();
    default:
      throw std::runtime_error("TreeRunner: invalid tick mode");
  }
}

BT::NodeStatus TreeRunner::tickOnce()
{
  return tree_.tickOnce();
}

BT::NodeStatus TreeRunner::tickExactlyOnce()
{
  return tree_.tickExactlyOnce();
}

BT::NodeStatus TreeRunner::tickWhileRunning(std::chrono::milliseconds max_block)
{
  return tree_.tickWhileRunning(max_block);
}

void TreeRunner::haltTree()
{
  tree_.haltTree();
}

void TreeRunner::reset()
{
  if (last_tree_xml_path_.empty())
  {
    throw std::runtime_error("TreeRunner::reset: no tree loaded yet");
  }

  tree_.haltTree();

  blackboard_ = BT::Blackboard::create();
  tree_ = factory_.createTreeFromFile(last_tree_xml_path_, blackboard_);
}

}  // namespace rlc_executive
