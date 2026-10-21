"""Check repository-relative Markdown links, excluding external URLs and anchors."""
from pathlib import Path
import re
root = Path(__file__).resolve().parents[1]
errors, count = [], 0
for p in [root/'README.md', root/'THIRD_PARTY_NOTICES.md', *sorted((root/'docs').glob('*.md'))]:
    for link in re.findall(r'\]\(([^)]+)\)', p.read_text()):
        link = link.split('#')[0]
        if not link or '://' in link or link.startswith('mailto:'):
            continue
        count += 1
        if not (p.parent/link).exists():
            errors.append(f'{p.relative_to(root)}: {link}')
if errors:
    raise SystemExit('\n'.join(errors))
print(f'{count} relative Markdown links verified')
