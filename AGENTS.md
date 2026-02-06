# Agent Guidelines (hri_rlc)

- Make request-scoped changes (no unrelated refactors).
- Commit messages: keep them short and professional; single-line, imperative mood; aim for <= 50 characters; no body unless absolutely necessary.
- Python: any new/modified function/method/class must include/update a docstring strictly per `docs/sphinx.md`.
- C++: any new/modified public API/function/method/class must include/update a Doxygen doc strictly per `docs/doxygen.md`. and refer to .clang-tidy rules follow them strictly. 
- CMakeLists: any new/modified CMakeLists.txt file should strictly follows the instrcutions in `docs/CMakeLists.md`.