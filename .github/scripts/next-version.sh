#!/usr/bin/env bash
# Prints the next release tag (vX.Y.Z) from the Conventional Commits since the
# last vX.Y.Z tag, and writes grouped release notes to $NOTES_FILE.
#   type!: … or BREAKING CHANGE:  → major
#   feat                           → minor
#   fix / perf / refactor          → patch
#   anything else (chore, ci, docs, …) → nothing to release, prints nothing
# With no previous tag, the first release is $INITIAL_VERSION (default 0.1.0).
set -euo pipefail

NOTES_FILE=${NOTES_FILE:-release-notes.md}
last=$(git describe --tags --abbrev=0 --match 'v[0-9]*.[0-9]*.[0-9]*' 2>/dev/null || true)

if [ -z "$last" ]; then
  echo "First release." > "$NOTES_FILE"
  echo "v${INITIAL_VERSION:-0.1.0}"
  exit 0
fi

breaking=() features=() fixes=()
while IFS=$'\x1f' read -r -d $'\x1e' sha subject body; do
  sha=${sha#$'\n'}
  type_re='^([a-z]+)(\(([^)]*)\))?(!)?: (.*)$'
  [[ $subject =~ $type_re ]] || continue
  type=${BASH_REMATCH[1]} scope=${BASH_REMATCH[3]} bang=${BASH_REMATCH[4]} desc=${BASH_REMATCH[5]}
  line="- ${scope:+**$scope:** }$desc (${sha:0:7})"
  if [ -n "$bang" ] || grep -qE '^BREAKING[ -]CHANGE:' <<<"$body"; then
    breaking+=("$line")
  elif [ "$type" = feat ]; then
    features+=("$line")
  elif [[ $type =~ ^(fix|perf|refactor)$ ]]; then
    fixes+=("$line")
  fi
done < <(git log --no-merges --format='%H%x1f%s%x1f%b%x1e' "$last..HEAD")

IFS=. read -r major minor patch <<<"${last#v}"
if   [ ${#breaking[@]} -gt 0 ]; then major=$((major + 1)) minor=0 patch=0
elif [ ${#features[@]} -gt 0 ]; then minor=$((minor + 1)) patch=0
elif [ ${#fixes[@]}    -gt 0 ]; then patch=$((patch + 1))
else exit 0
fi

{
  [ ${#breaking[@]} -gt 0 ] && printf '## Breaking changes\n%s\n\n' "$(printf '%s\n' "${breaking[@]}")"
  [ ${#features[@]} -gt 0 ] && printf '## Features\n%s\n\n' "$(printf '%s\n' "${features[@]}")"
  [ ${#fixes[@]}    -gt 0 ] && printf '## Fixes & improvements\n%s\n\n' "$(printf '%s\n' "${fixes[@]}")"
  printf '**Full changelog:** %s...v%s.%s.%s\n' "$last" "$major" "$minor" "$patch"
} > "$NOTES_FILE"

echo "v$major.$minor.$patch"
