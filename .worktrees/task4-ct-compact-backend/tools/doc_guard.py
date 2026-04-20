#!/usr/bin/env python3
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
DOCS = {
    "changes": "docs/CHANGES.md",
    "trace": "docs/TRACEABILITY.md",
}

CODE_PREFIXES = ("src/", "include/", "apps/", "tests/", "launch/", "config/")
DOC_FILES = (DOCS["changes"], DOCS["trace"])

def sh(cmd):
    return subprocess.check_output(cmd, cwd=REPO_ROOT).decode().strip()

def staged_files():
    out = sh(["git", "diff", "--cached", "--name-only"])
    return [x for x in out.splitlines() if x.strip()]

def is_code_change(path: str) -> bool:
    if path.startswith("docs/"):
        return False
    return path.startswith(CODE_PREFIXES)

def main():
    files = staged_files()
    if not files:
        return 0

    has_code = any(is_code_change(f) for f in files)
    if not has_code:
        return 0

    changed_docs = set(f for f in files if f in DOC_FILES)

    missing = [p for p in DOC_FILES if p not in changed_docs]
    if missing:
        print("\n[DOC-GUARD] 你提交里包含代码改动，但缺少必需的文档同步更新：")
        for m in missing:
            print(f"  - 必须更新: {m}")
        print("\n解决办法：")
        print("  1) 更新 docs/CHANGES.md：写清楚做了什么/为什么/对应 IAP-RQ")
        print("  2) 更新 docs/TRACEABILITY.md：补齐需求↔实现↔验证映射")
        print("  3) 然后重新 git add 这两个文件再提交\n")
        return 1

    return 0

if __name__ == "__main__":
    sys.exit(main())