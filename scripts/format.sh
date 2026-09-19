#!/usr/bin/env sh
set -eu

mode="${1:-fix}"

case "$mode" in
  fix | --fix)
    clang_format_args="-i"
    ;;
  check | --check)
    clang_format_args="--dry-run --Werror"
    ;;
  *)
    echo "Usage: $0 [fix|check]" >&2
    exit 2
    ;;
esac

# Output drifts between clang-format majors, so one is pinned.
CLANG_FORMAT="${CLANG_FORMAT:-clang-format}"
major=$("$CLANG_FORMAT" --version | sed -n 's/.*clang-format version \([0-9]*\).*/\1/p')
if [ "$major" != "21" ]; then
  echo "clang-format 21 required (pip install 'clang-format==21.1.*'), found: $("$CLANG_FORMAT" --version)" >&2
  exit 1
fi

files=$(
  find include src -type f \( -name '*.c' -o -name '*.h' \)
  find test -maxdepth 1 -type f -name '*.c'
)

if [ -z "$files" ]; then
  exit 0
fi

# shellcheck disable=SC2086
"$CLANG_FORMAT" $clang_format_args $files
