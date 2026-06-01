#!/usr/bin/env python3
"""
Translate all Markdown files under docs/ja/ into English and write them under docs/en/.

Examples:
  python translate_docs.py
  python translate_docs.py --src docs/ja --dst docs/en --model gpt-5.4
"""

from __future__ import annotations

import argparse
import os
import re
import sys
import time
from pathlib import Path
from typing import Dict, List, Tuple

from openai import OpenAI

FRONT_MATTER_RE = re.compile(r"\A(---\n.*?\n---\n?)", re.DOTALL)
FENCED_CODE_RE = re.compile(
    r"(^```[^\n]*\n.*?^```[ \t]*$|^~~~[^\n]*\n.*?^~~~[ \t]*$)",
    re.MULTILINE | re.DOTALL,
)
INLINE_CODE_RE = re.compile(r"(`+)([^`\n]|[^`\n].*?[^`\n])\1")
HTML_COMMENT_RE = re.compile(r"<!--.*?-->", re.DOTALL)

TRANSLATION_INSTRUCTIONS = """\
You are a professional technical translator.
Translate Japanese Markdown into natural, concise English.

Rules:
- Preserve Markdown structure exactly: headings, lists, tables, blockquotes, admonitions, checklists.
- Preserve placeholders like ⟪PH_0001⟫ exactly.
- Do not change fenced code blocks, inline code, HTML comments, URLs, anchors, file paths, command lines, environment variables, API names, class/function names, or product names unless they are natural-language prose.
- Keep ROS 2, C++, Python, CMake, Tobas, UADF, PX4, ArduPilot, GitHub Pages, Read the Docs, and similar names unchanged.
- Keep relative links and image paths unchanged.
- Return only the translated Markdown. No explanation.
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--src", type=Path, default=Path("docs/ja"))
    parser.add_argument("--dst", type=Path, default=Path("docs/en"))
    parser.add_argument("--model", default="gpt-5.4")
    parser.add_argument("--chunk-chars", type=int, default=6000)
    parser.add_argument("--retries", type=int, default=3)
    parser.add_argument("--sleep", type=float, default=1.5)
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite destination files even if they already exist.",
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
    Mask code blocks, inline code, and HTML comments so the translator won't touch them.
    """
    placeholders: Dict[str, str] = {}
    counter = 0

    def make_replacer(prefix: str):
        def replacer(match: re.Match[str]) -> str:
            nonlocal counter
            key = f"⟪{prefix}_{counter:04d}⟫"
            placeholders[key] = match.group(0)
            counter += 1
            return key

        return replacer

    text = FENCED_CODE_RE.sub(make_replacer("PH"), text)
    text = HTML_COMMENT_RE.sub(make_replacer("PH"), text)
    text = INLINE_CODE_RE.sub(make_replacer("PH"), text)
    return text, placeholders


def unmask_segments(text: str, placeholders: Dict[str, str]) -> str:
    for key, value in placeholders.items():
        text = text.replace(key, value)
    return text


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
        if len(current) + len(part) <= max_chars:
            current += part
            continue
        if current:
            chunks.append(current)
            current = part
        else:
            # Fallback for a single huge block
            for i in range(0, len(part), max_chars):
                chunks.append(part[i : i + max_chars])
            current = ""

    if current:
        chunks.append(current)

    return chunks


def translate_chunk(
    client: OpenAI,
    text: str,
    model: str,
    retries: int,
    sleep_sec: float,
) -> str:
    last_error: Exception | None = None

    for attempt in range(1, retries + 1):
        try:
            response = client.responses.create(
                model=model,
                instructions=TRANSLATION_INSTRUCTIONS,
                input=text,
            )
            output = response.output_text
            if not output:
                raise RuntimeError("Empty translation response.")
            return output
        except Exception as exc:
            last_error = exc
            if attempt == retries:
                break
            time.sleep(sleep_sec * attempt)

    raise RuntimeError(f"Translation failed after {retries} attempts: {last_error}")


def normalize_newlines(text: str) -> str:
    return text.replace("\r\n", "\n").replace("\r", "\n")


def translate_markdown(
    client: OpenAI,
    src_text: str,
    model: str,
    chunk_chars: int,
    retries: int,
    sleep_sec: float,
) -> str:
    src_text = normalize_newlines(src_text)
    front_matter, body = split_front_matter(src_text)

    masked_body, placeholders = mask_segments(body)
    chunks = chunk_text(masked_body, chunk_chars)

    translated_chunks: List[str] = []
    for idx, chunk in enumerate(chunks, start=1):
        print(f"    chunk {idx}/{len(chunks)}")
        translated = translate_chunk(
            client=client,
            text=chunk,
            model=model,
            retries=retries,
            sleep_sec=sleep_sec,
        )
        translated_chunks.append(translated)

    translated_body = "".join(translated_chunks)
    translated_body = unmask_segments(translated_body, placeholders)

    return front_matter + translated_body


def iter_markdown_files(root: Path) -> List[Path]:
    return sorted(p for p in root.rglob("*.md") if p.is_file())


def main() -> int:
    args = parse_args()

    if not args.src.exists():
        print(f"Source directory does not exist: {args.src}", file=sys.stderr)
        return 1

    if not os.environ.get("OPENAI_API_KEY"):
        print("OPENAI_API_KEY is not set.", file=sys.stderr)
        return 1

    client = OpenAI()

    files = iter_markdown_files(args.src)
    if not files:
        print(f"No Markdown files found under: {args.src}")
        return 0

    for src_path in files:
        rel = src_path.relative_to(args.src)
        dst_path = args.dst / rel

        if dst_path.exists() and not args.overwrite:
            print(f"Skip (already exists): {dst_path}")
            continue

        print(f"Translate: {src_path} -> {dst_path}")
        src_text = src_path.read_text(encoding="utf-8")
        translated = translate_markdown(
            client=client,
            src_text=src_text,
            model=args.model,
            chunk_chars=args.chunk_chars,
            retries=args.retries,
            sleep_sec=args.sleep,
        )

        dst_path.parent.mkdir(parents=True, exist_ok=True)
        dst_path.write_text(translated, encoding="utf-8")

    print("Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
