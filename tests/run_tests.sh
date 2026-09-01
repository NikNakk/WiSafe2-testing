#!/bin/sh
set -eu

repo_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
test_dir=$(mktemp -d "${TMPDIR:-/tmp}/wisafe2-tests.XXXXXX")
trap 'rm -rf "$test_dir"' EXIT HUP INT TERM

${CXX:-c++} \
  -std=c++17 \
  -Wall \
  -Wextra \
  -Werror \
  -I"$repo_dir/components/wisafe2" \
  "$repo_dir/tests/protocol_test.cpp" \
  "$repo_dir/components/wisafe2/protocol.cpp" \
  -o "$test_dir/protocol_test"

"$test_dir/protocol_test"

${CXX:-c++} \
  -std=c++17 \
  -Wall \
  -Wextra \
  -Werror \
  -I"$repo_dir/components/wisafe2" \
  "$repo_dir/tests/radio_transport_test.cpp" \
  "$repo_dir/components/wisafe2/radio_transport.cpp" \
  "$repo_dir/components/wisafe2/protocol.cpp" \
  -o "$test_dir/radio_transport_test"

"$test_dir/radio_transport_test"
