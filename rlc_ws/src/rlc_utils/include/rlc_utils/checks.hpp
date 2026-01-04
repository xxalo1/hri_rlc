#pragma once

#include <Eigen/Core>

#include <cstddef>

namespace rlc_utils::checks {

template <typename T>
[[nodiscard]] inline std::size_t size_of(const T& v) noexcept {
  return static_cast<std::size_t>(v.size());
}

template <typename T>
[[nodiscard]] inline bool size_is(const T& v, std::size_t n) noexcept {
  return size_of(v) == n;
}

template <typename... Ts>
[[nodiscard]] inline bool all_size_is(std::size_t n, const Ts&... vs) noexcept {
  return (... && size_is(vs, n));
}

template <typename First, typename... Rest>
[[nodiscard]] inline bool all_same_size(const First& first,
                                        const Rest&... rest) noexcept {
  const auto n = size_of(first);
  return (... && size_is(rest, n));
}

template <typename Container>
[[nodiscard]] inline bool empty_or_size_is(const Container& c,
                                           std::size_t n) noexcept {
  return c.empty() || size_is(c, n);
}

template <typename... Ts>
[[nodiscard]] inline bool all_empty_or_size_is(std::size_t n,
                                              const Ts&... vs) noexcept {
  return (... && empty_or_size_is(vs, n));
}

template <typename Derived>
[[nodiscard]] inline bool shape_is(const Eigen::MatrixBase<Derived>& m,
                                   Eigen::Index rows,
                                   Eigen::Index cols) noexcept {
  return (m.rows() == rows) && (m.cols() == cols);
}

}  // namespace rlc_utils::checks
