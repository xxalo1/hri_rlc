#include "rlc_executive/bt/tree_runner.hpp"

#include <fstream>
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
    // Trim simple whitespace
    while (!line.empty() &&
           (line.back() == ' ' || line.back() == '\t' || line.back() == '\r'))
      line.pop_back();
    std::size_t i = 0;
    while (i < line.size() && (line[i] == ' ' || line[i] == '\t'))
      ++i;
    line = line.substr(i);

    if (line.empty())
      continue;
    if (line[0] == '#')
      continue;

    libs.push_back(line);
  }

  return libs;
}

}  // namespace

void TreeRunner::configure(const std::string& plugins_path_or_list_file)
{
  std::vector<std::string> libs;

  if (looksLikeSharedLibraryPath(plugins_path_or_list_file))
  {
    libs.push_back(plugins_path_or_list_file);
  }
  else
  {
    libs = loadLibraryListFile(plugins_path_or_list_file);
  }

  for (const auto& lib : libs)
  {
    factory_.registerFromPlugin(lib);
  }
}

void TreeRunner::loadTreeFromXmlFile(const std::string& tree_xml_path,
                                     const std::shared_ptr<RuntimeContext>& ctx)
{
  blackboard_ = BT::Blackboard::create();
  blackboard_->set<std::shared_ptr<RuntimeContext>>(bb::RUNTIME_CONTEXT, ctx);

  tree_ = factory_.createTreeFromFile(tree_xml_path, blackboard_);
}

BT::NodeStatus TreeRunner::tickOnce()
{
  return tree_.tickRoot();
}

void TreeRunner::haltTree()
{
  tree_.haltTree();
}

BT::Blackboard::Ptr TreeRunner::blackboard() const
{
  return blackboard_;
}

}  // namespace rlc_executive