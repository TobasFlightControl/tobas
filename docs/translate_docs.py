#!/usr/bin/env python3
"""
Translate all Markdown files under docs/ja/ into English and write them under docs/en/.
"""

from __future__ import annotations

import argparse
import difflib
import os
import re
import secrets
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import TYPE_CHECKING, Dict, List, Sequence, Tuple

if TYPE_CHECKING:
    from openai import OpenAI

FRONT_MATTER_RE = re.compile(r"\A(---\n.*?\n---\n?)", re.DOTALL)
FENCED_CODE_RE = re.compile(
    r"(^[ \t]*(?P<fence>`{3,}|~{3,})[^\n]*\n.*?^[ \t]*(?P=fence)[ \t]*$)",
    re.MULTILINE | re.DOTALL,
)
INLINE_CODE_RE = re.compile(r"(`+)([^`\n]|[^`\n].*?[^`\n])\1")
HTML_COMMENT_RE = re.compile(r"<!--.*?-->", re.DOTALL)
HTML_TAG_RE = re.compile(r"</?[A-Za-z][^>\n]*>|<!DOCTYPE[^>\n]*>", re.IGNORECASE)
MARKDOWN_LINK_DEST_RE = re.compile(
    r"(?P<prefix>!?\[[^\]\n]*\]\(\s*)"
    r"(?P<destination><[^>\n]+>|[^\s)]+)"
    r"(?P<suffix>\s*(?:(?:\"[^\"\n]*\"|'[^'\n]*'|\([^\n)]*\))\s*)?\))"
)
REFERENCE_LINK_DEST_RE = re.compile(
    r"(?P<prefix>^[ \t]{0,3}\[[^\]\n]+\]:[ \t]*)" r"(?P<destination><[^>\n]+>|\S+)",
    re.MULTILINE,
)
URL_RE = re.compile(r"https?://[^\s<>\"]+")
PLACEHOLDER_RE = re.compile(r"⟪TOBAS_[A-F0-9]{16}_\d{4,}⟫")
CONTEXT_CHARS = 1200

TRANSLATION_INSTRUCTIONS = """\
You are a professional technical translator.
Translate Japanese Markdown into natural, concise English.

Rules:
- Preserve Markdown structure exactly: headings, lists, tables, blockquotes, admonitions, checklists.
- Preserve every placeholder beginning with ⟪TOBAS_ exactly and exactly once.
- Do not change fenced code blocks, inline code, HTML comments, URLs, anchors, file paths, command lines, environment variables, API names, class/function names, or product names unless they are natural-language prose.
- Keep ROS 2, C++, Python, CMake, Tobas, UADF, PX4, ArduPilot, GitHub Pages, Read the Docs, and similar names unchanged.
- Keep relative links and image paths unchanged.
- Use terminology consistently with the surrounding context.
- Translate only the text inside <SOURCE_TO_TRANSLATE>. The other XML sections are context only and must not appear in the output.
- Return only the translated Markdown. No explanation.
"""


class TranslationValidationError(RuntimeError):
    """Raised when a translation changes protected Markdown content or structure."""


def positive_int(value: str) -> int:
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return parsed


def nonnegative_float(value: str) -> float:
    parsed = float(value)
    if parsed < 0:
        raise argparse.ArgumentTypeError("must be zero or greater")
    return parsed


def parse_args() -> argparse.Namespace:
    docs_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument("--src", type=Path, default=docs_dir / "docs/ja")
    parser.add_argument("--dst", type=Path, default=docs_dir / "docs/en")
    parser.add_argument("--model", default="gpt-5.6-sol")
    parser.add_argument(
        "--reasoning-effort",
        "--reasoning",
        metavar="LEVEL",
        help=(
            "Reasoning effort passed to the Responses API, for example low, "
            "medium, or high. If omitted, the model default is used."
        ),
    )
    parser.add_argument("--chunk-chars", type=positive_int, default=6000)
    parser.add_argument("--retries", type=positive_int, default=1)
    parser.add_argument("--sleep", type=nonnegative_float, default=1.5)
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite destination files even if they already exist.",
    )
    parser.add_argument(
        "--changed-only",
        action="store_true",
        help=(
            "Translate only Markdown files changed under --src. Existing "
            "destination files are updated block-by-block from the Japanese diff."
        ),
    )
    parser.add_argument(
        "--base-ref",
        default="HEAD",
        help="Git revision used as the base for --changed-only.",
    )
    parser.add_argument(
        "--allow-full-fallback",
        action="store_true",
        help=("Allow --changed-only to replace a whole English file when its " "blocks cannot be aligned safely."),
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print generated diffs without writing destination files.",
    )
    return parser.parse_args()


def split_front_matter(text: str) -> Tuple[str, str]:
    """
    Return (front_matter, body).
    front_matter includes the trailing newline after the closing --- if present.
    """
    match = FRONT_MATTER_RE.match(text)
    if not match:
        return "", text
    front_matter = match.group(1)
    body = text[len(front_matter) :]
    return front_matter, body


def mask_segments(text: str) -> Tuple[str, Dict[str, str]]:
    """
    Mask content that must remain byte-for-byte identical after translation.
    """
    placeholders: Dict[str, str] = {}
    counter = 0
    namespace = f"TOBAS_{secrets.token_hex(8).upper()}"
    while namespace in text:
        namespace = f"TOBAS_{secrets.token_hex(8).upper()}"

    def make_placeholder(value: str) -> str:
        nonlocal counter
        key = f"⟪{namespace}_{counter:04d}⟫"
        placeholders[key] = value
        counter += 1
        return key

    def make_replacer(_prefix: str):
        def replacer(match: re.Match[str]) -> str:
            return make_placeholder(match.group(0))

        return replacer

    def replace_destination(match: re.Match[str]) -> str:
        return (
            match.group("prefix") + make_placeholder(match.group("destination")) + match.groupdict().get("suffix", "")
        )

    text = FENCED_CODE_RE.sub(make_replacer("PH"), text)
    text = HTML_COMMENT_RE.sub(make_replacer("PH"), text)
    text = INLINE_CODE_RE.sub(make_replacer("PH"), text)
    text = MARKDOWN_LINK_DEST_RE.sub(replace_destination, text)
    text = REFERENCE_LINK_DEST_RE.sub(replace_destination, text)
    text = HTML_TAG_RE.sub(make_replacer("PH"), text)
    text = URL_RE.sub(make_replacer("PH"), text)
    return text, placeholders


def unmask_segments(text: str, placeholders: Dict[str, str]) -> str:
    for key, value in placeholders.items():
        count = text.count(key)
        if count != 1:
            raise TranslationValidationError(f"Expected placeholder {key} exactly once, found {count} times.")
        text = text.replace(key, value)
    return text


def markdown_structure(text: str) -> List[str]:
    """Return the ordered Markdown constructs that translation must preserve."""
    structure: List[str] = []
    for line in text.splitlines():
        stripped = line.lstrip()
        indent = len(line) - len(stripped)

        heading = re.match(r"(#{1,6})\s", stripped)
        if heading:
            structure.append(f"heading:{len(heading.group(1))}")
            continue
        if re.match(r"!!!\s+\S+", stripped):
            structure.append(f"admonition:{indent}")
            continue
        if stripped.startswith(">"):
            structure.append(f"blockquote:{indent}")
            continue
        if re.match(r"[-+*]\s+", stripped):
            structure.append(f"unordered-list:{indent}")
            continue
        if re.match(r"\d+[.)]\s+", stripped):
            structure.append(f"ordered-list:{indent}")
            continue
        if "|" in stripped:
            if re.match(r"\|?[ :|-]+\|[ :|-]*\|?$", stripped) and "-" in stripped:
                structure.append(f"table-separator:{stripped.count('|')}")
            else:
                structure.append(f"table-row:{stripped.count('|')}")

    return structure


def split_oversized_text(text: str, max_chars: int) -> List[str]:
    """Split a large Markdown block without cutting protected placeholders."""
    atoms = re.split(f"({PLACEHOLDER_RE.pattern})", text)
    pieces: List[str] = []
    current = ""

    def append_piece(piece: str) -> None:
        nonlocal current
        if not piece:
            return
        if len(current) + len(piece) <= max_chars:
            current += piece
            return
        if current:
            pieces.append(current)
            current = ""
        if len(piece) <= max_chars:
            current = piece
            return
        if PLACEHOLDER_RE.fullmatch(piece):
            raise ValueError("--chunk-chars is too small to preserve protected Markdown content.")
        for start in range(0, len(piece), max_chars):
            segment = piece[start : start + max_chars]
            if len(segment) == max_chars:
                pieces.append(segment)
            else:
                current = segment

    for atom in atoms:
        if not atom:
            continue
        if PLACEHOLDER_RE.fullmatch(atom):
            append_piece(atom)
            continue
        for token in re.findall(r"\S+\s*|\s+", atom):
            append_piece(token)

    if current:
        pieces.append(current)
    return pieces


def chunk_text(text: str, max_chars: int) -> List[str]:
    """
    Chunk markdown by paragraph-ish boundaries while preserving separators.
    """
    if len(text) <= max_chars:
        return [text]

    parts = re.split(r"(\n(?=#+\s)|\n{2,})", text)
    chunks: List[str] = []
    current = ""

    for part in parts:
        if not part:
            continue
        if len(part) > max_chars:
            if current:
                chunks.append(current)
                current = ""
            chunks.extend(split_oversized_text(part, max_chars))
            continue
        if len(current) + len(part) <= max_chars:
            current += part
            continue
        if current:
            chunks.append(current)
        current = part

    if current:
        chunks.append(current)

    if "".join(chunks) != text:
        raise RuntimeError("Internal error: chunking changed the source text.")
    return chunks


def translate_chunk(
    client: OpenAI,
    text: str,
    model: str,
    reasoning_effort: str | None,
    retries: int,
    sleep_sec: float,
    previous_translation: str = "",
    following_source: str = "",
) -> str:
    last_error: Exception | None = None
    request_input = (
        "Translate only the Markdown in <SOURCE_TO_TRANSLATE>. "
        "Use the other sections only to keep terminology and style consistent.\n\n"
        f"<PREVIOUS_TRANSLATION>\n{previous_translation}\n</PREVIOUS_TRANSLATION>\n\n"
        f"<SOURCE_TO_TRANSLATE>\n{text}\n</SOURCE_TO_TRANSLATE>\n\n"
        f"<FOLLOWING_SOURCE>\n{following_source}\n</FOLLOWING_SOURCE>"
    )

    for attempt in range(1, retries + 1):
        try:
            reasoning_options = {"reasoning": {"effort": reasoning_effort}} if reasoning_effort is not None else {}
            response = client.responses.create(
                model=model,
                instructions=TRANSLATION_INSTRUCTIONS,
                input=request_input,
                **reasoning_options,
            )
            output = response.output_text
            if not output:
                raise TranslationValidationError("The API returned an empty translation.")
            return output
        except Exception as exc:
            last_error = exc
            status_code = getattr(exc, "status_code", None)
            retryable = isinstance(exc, TranslationValidationError)
            retryable = retryable or status_code in {408, 409, 429}
            retryable = retryable or isinstance(status_code, int) and status_code >= 500
            retryable = retryable or type(exc).__name__ in {
                "APIConnectionError",
                "APITimeoutError",
                "RateLimitError",
            }
            if not retryable:
                raise RuntimeError(f"Translation failed: {exc}") from exc
            if attempt == retries:
                break
            retry_after = None
            response_headers = getattr(getattr(exc, "response", None), "headers", {})
            if response_headers:
                try:
                    retry_after = float(response_headers.get("retry-after"))
                except (TypeError, ValueError):
                    pass
            delay = retry_after if retry_after is not None else sleep_sec * (2 ** (attempt - 1))
            time.sleep(delay)

    raise RuntimeError(f"Translation failed after {retries} attempts: {last_error}")


def normalize_newlines(text: str) -> str:
    return text.replace("\r\n", "\n").replace("\r", "\n")


def run_git(args: Sequence[str], cwd: Path) -> str:
    result = subprocess.run(
        ["git", *args],
        cwd=cwd,
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    return result.stdout


def find_git_root(path: Path) -> Path:
    output = run_git(["rev-parse", "--show-toplevel"], path)
    return Path(output.strip())


def to_git_path(path: Path, git_root: Path) -> str:
    return path.resolve().relative_to(git_root.resolve()).as_posix()


def read_file_from_git(path: Path, base_ref: str, git_root: Path) -> str | None:
    git_path = to_git_path(path, git_root)
    try:
        return run_git(["show", f"{base_ref}:{git_path}"], git_root)
    except subprocess.CalledProcessError:
        return None


def changed_markdown_files(root: Path, base_ref: str) -> List[Path]:
    git_root = find_git_root(root)
    root_path = to_git_path(root, git_root)

    changed = set(run_git(["diff", "--name-only", base_ref, "--", root_path], git_root).splitlines())
    changed.update(
        run_git(
            ["ls-files", "--others", "--exclude-standard", "--", root_path],
            git_root,
        ).splitlines()
    )

    files = []
    for git_path in sorted(changed):
        path = git_root / git_path
        if path.suffix == ".md" and path.is_file() and path.name != "README.md":
            files.append(path)
    return files


def split_markdown_blocks(text: str) -> List[str]:
    """
    Split Markdown into paragraph-ish blocks while keeping separators.
    """
    lines = text.splitlines(keepends=True)
    blocks: List[str] = []
    current: List[str] = []
    fence: str | None = None

    for line in lines:
        stripped = line.lstrip()
        fence_match = re.match(r"(```|~~~)", stripped)

        current.append(line)

        if fence:
            if stripped.startswith(fence):
                fence = None
            continue

        if fence_match:
            fence = fence_match.group(1)
            continue

        if not line.strip():
            blocks.append("".join(current))
            current = []

    if current:
        blocks.append("".join(current))

    return blocks


def masked_markdown_structure(text: str) -> List[str]:
    masked, _ = mask_segments(text)
    return markdown_structure(masked)


def validate_block_alignment(base_blocks: Sequence[str], dst_blocks: Sequence[str]) -> None:
    if len(base_blocks) != len(dst_blocks):
        raise ValueError(
            "Cannot align Japanese base blocks with existing English blocks "
            f"({len(base_blocks)} != {len(dst_blocks)})."
        )

    for index, (base_block, dst_block) in enumerate(zip(base_blocks, dst_blocks), start=1):
        if masked_markdown_structure(base_block) != masked_markdown_structure(dst_block):
            raise ValueError(f"Markdown structure differs in aligned block {index}.")


def markdown_link_destinations(text: str) -> List[str]:
    destinations = [match.group("destination") for match in MARKDOWN_LINK_DEST_RE.finditer(text)]
    destinations.extend(match.group("destination") for match in REFERENCE_LINK_DEST_RE.finditer(text))
    return destinations


def translate_markdown_diff(
    client: OpenAI,
    base_src_text: str,
    src_text: str,
    dst_text: str,
    model: str,
    reasoning_effort: str | None,
    chunk_chars: int,
    retries: int,
    sleep_sec: float,
) -> str:
    src_text = normalize_newlines(src_text)
    base_src_text = normalize_newlines(base_src_text)
    dst_text = normalize_newlines(dst_text)

    src_front_matter, src_body = split_front_matter(src_text)
    base_front_matter, base_body = split_front_matter(base_src_text)
    dst_front_matter, dst_body = split_front_matter(dst_text)

    base_blocks = split_markdown_blocks(base_body)
    src_blocks = split_markdown_blocks(src_body)
    dst_blocks = split_markdown_blocks(dst_body)

    validate_block_alignment(base_blocks, dst_blocks)

    front_matter = dst_front_matter
    if src_front_matter != base_front_matter:
        front_matter = src_front_matter

    result_blocks: List[str] = []
    matcher = difflib.SequenceMatcher(a=base_blocks, b=src_blocks, autojunk=False)

    for tag, i1, i2, j1, j2 in matcher.get_opcodes():
        if tag == "equal":
            result_blocks.extend(dst_blocks[i1:i2])
            continue

        if tag == "delete":
            print(f"    delete blocks {i1 + 1}-{i2}")
            continue

        changed_text = "".join(src_blocks[j1:j2])
        print(f"    translate changed blocks {j1 + 1}-{j2}")
        result_blocks.append(
            translate_markdown(
                client=client,
                src_text=changed_text,
                model=model,
                reasoning_effort=reasoning_effort,
                chunk_chars=chunk_chars,
                retries=retries,
                sleep_sec=sleep_sec,
                context_before="".join(dst_blocks[max(0, i1 - 2) : i1]),
                context_after="".join(src_blocks[j2 : j2 + 2]),
            )
        )

    translated_body = "".join(result_blocks)
    return front_matter + translated_body


def translate_markdown(
    client: OpenAI,
    src_text: str,
    model: str,
    reasoning_effort: str | None,
    chunk_chars: int,
    retries: int,
    sleep_sec: float,
    context_before: str = "",
    context_after: str = "",
) -> str:
    src_text = normalize_newlines(src_text)
    front_matter, body = split_front_matter(src_text)

    masked_body, placeholders = mask_segments(body)
    chunks = chunk_text(masked_body, chunk_chars)

    translated_chunks: List[str] = []
    for idx, chunk in enumerate(chunks, start=1):
        print(f"    chunk {idx}/{len(chunks)}")
        previous_translation = translated_chunks[-1] if translated_chunks else context_before
        following_source = chunks[idx] if idx < len(chunks) else context_after
        translated = translate_chunk(
            client=client,
            text=chunk,
            model=model,
            reasoning_effort=reasoning_effort,
            retries=retries,
            sleep_sec=sleep_sec,
            previous_translation=previous_translation[-CONTEXT_CHARS:],
            following_source=following_source[:CONTEXT_CHARS],
        )
        translated_chunks.append(translated)

    translated_body = "".join(translated_chunks)
    translated_body = unmask_segments(translated_body, placeholders)

    return front_matter + translated_body


def iter_markdown_files(root: Path) -> List[Path]:
    return sorted(p for p in root.rglob("*.md") if p.is_file() and p.name != "README.md")


def deleted_markdown_files(root: Path, base_ref: str) -> List[Path]:
    git_root = find_git_root(root)
    root_path = to_git_path(root, git_root)
    deleted = run_git(
        ["diff", "--diff-filter=D", "--name-only", base_ref, "--", root_path],
        git_root,
    ).splitlines()
    return [git_root / path for path in deleted if Path(path).suffix == ".md" and Path(path).name != "README.md"]


def validate_paths(src: Path, dst: Path) -> None:
    if not src.is_dir():
        raise ValueError(f"Source directory does not exist or is not a directory: {src}")

    src_resolved = src.resolve()
    dst_resolved = dst.resolve()
    if src_resolved == dst_resolved:
        raise ValueError("Source and destination directories must be different.")
    if src_resolved in dst_resolved.parents or dst_resolved in src_resolved.parents:
        raise ValueError("Source and destination directories must not contain one another.")


def write_text_atomic(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    file_descriptor, temp_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent, text=True)
    temp_path = Path(temp_name)
    try:
        with os.fdopen(file_descriptor, "w", encoding="utf-8", newline="") as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp_path, path)
    finally:
        temp_path.unlink(missing_ok=True)


def print_generated_diff(path: Path, translated: str) -> None:
    current = path.read_text(encoding="utf-8") if path.exists() else ""
    diff = difflib.unified_diff(
        current.splitlines(keepends=True),
        translated.splitlines(keepends=True),
        fromfile=str(path),
        tofile=f"{path} (generated)",
    )
    sys.stdout.writelines(diff)


def main() -> int:
    args = parse_args()

    try:
        validate_paths(args.src, args.dst)
    except ValueError as exc:
        print(exc, file=sys.stderr)
        return 2

    files = changed_markdown_files(args.src, args.base_ref) if args.changed_only else iter_markdown_files(args.src)
    deleted_files = deleted_markdown_files(args.src, args.base_ref) if args.changed_only else []
    for deleted_path in deleted_files:
        rel = deleted_path.resolve().relative_to(args.src.resolve())
        stale_dst = args.dst / rel
        if stale_dst.exists():
            print(
                f"Warning: source was deleted but destination still exists: {stale_dst}",
                file=sys.stderr,
            )

    if not files:
        if args.changed_only:
            print(f"No changed Markdown files found under: {args.src}")
        else:
            print(f"No Markdown files found under: {args.src}")
        return 0

    if not os.environ.get("OPENAI_API_KEY"):
        print("OPENAI_API_KEY is not set.", file=sys.stderr)
        return 1

    try:
        from openai import OpenAI
    except ImportError:
        print("The openai package is not installed. Run: pip install openai", file=sys.stderr)
        return 1

    client = OpenAI()

    git_root = find_git_root(args.src) if args.changed_only else None

    for src_path in files:
        rel = src_path.resolve().relative_to(args.src.resolve())
        dst_path = args.dst / rel

        if not args.changed_only and dst_path.exists() and not args.overwrite:
            print(f"Skip (already exists): {dst_path}")
            continue

        print(f"Translate: {src_path} -> {dst_path}")
        src_text = src_path.read_text(encoding="utf-8")

        translated = None
        if args.changed_only and dst_path.exists() and git_root is not None:
            base_src_text = read_file_from_git(src_path, args.base_ref, git_root)
            if base_src_text is not None:
                dst_text = dst_path.read_text(encoding="utf-8")
                try:
                    translated = translate_markdown_diff(
                        client=client,
                        base_src_text=base_src_text,
                        src_text=src_text,
                        dst_text=dst_text,
                        model=args.model,
                        reasoning_effort=args.reasoning_effort,
                        chunk_chars=args.chunk_chars,
                        retries=args.retries,
                        sleep_sec=args.sleep,
                    )
                except ValueError as exc:
                    if not args.allow_full_fallback:
                        print(
                            f"Cannot update {dst_path} safely: {exc}\n"
                            "Re-run with --allow-full-fallback to translate the whole file.",
                            file=sys.stderr,
                        )
                        return 1
                    print(f"    fallback to full translation: {exc}")

        if translated is None:
            translated = translate_markdown(
                client=client,
                src_text=src_text,
                model=args.model,
                reasoning_effort=args.reasoning_effort,
                chunk_chars=args.chunk_chars,
                retries=args.retries,
                sleep_sec=args.sleep,
            )

        if args.dry_run:
            print_generated_diff(dst_path, translated)
        else:
            write_text_atomic(dst_path, translated)

    print("Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
