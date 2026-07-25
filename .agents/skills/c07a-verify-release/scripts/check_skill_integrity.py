#!/usr/bin/env python3
from pathlib import Path
import re
import sys


FRONTMATTER_RE = re.compile(r"\A---\r?\n(.*?)\r?\n---\r?\n", re.S)
LINK_RE = re.compile(r"(?:\]\(|`)(references/[^`)`]+)")


def parse_frontmatter(text: str) -> dict:
    match = FRONTMATTER_RE.match(text)
    if not match:
        return {}
    data = {}
    for line in match.group(1).splitlines():
        if ":" in line:
            key, value = line.split(":", 1)
            data[key.strip()] = value.strip().strip('"')
    return data


def main() -> int:
    root = Path(__file__).resolve().parents[4]
    skills_dir = root / ".agents" / "skills"
    if not skills_dir.is_dir():
        print("ERROR: missing .agents/skills")
        return 2

    errors = []
    names = {}

    for skill_dir in sorted(p for p in skills_dir.iterdir() if p.is_dir()):
        skill_md = skill_dir / "SKILL.md"
        if not skill_md.is_file():
            errors.append(f"{skill_dir.name}: missing SKILL.md")
            continue
        text = skill_md.read_text(encoding="utf-8", errors="replace")
        fm = parse_frontmatter(text)
        name = fm.get("name")
        desc = fm.get("description")
        if not name:
            errors.append(f"{skill_dir.name}: missing frontmatter name")
        elif name != skill_dir.name:
            errors.append(f"{skill_dir.name}: name does not match directory ({name})")
        else:
            names.setdefault(name, []).append(skill_dir)
        if not desc:
            errors.append(f"{skill_dir.name}: missing frontmatter description")

        for rel in LINK_RE.findall(text):
            target = skill_dir / rel
            if not target.is_file():
                errors.append(f"{skill_dir.name}: missing linked reference {rel}")

    for name, dirs in names.items():
        if len(dirs) > 1:
            errors.append(f"duplicate skill name {name}: {', '.join(str(d) for d in dirs)}")

    if errors:
        print("FAIL skill integrity")
        for error in errors:
            print(f"ERROR: {error}")
        return 1

    print("PASS skill integrity")
    for name in sorted(names):
        print(f"  {name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
