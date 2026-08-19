# Tobas Documentation

This directory contains the MkDocs project used to build the Tobas user guide.
It is intended for people editing, translating, previewing, or publishing the documentation.
If you want to install or use Tobas, start from the published guide:

- https://tobasflightcontrol.github.io/tobas/latest/

## Structure

- `docs/ja`: Japanese source documentation.
- `docs/en`: English documentation generated or maintained from the Japanese source.
- `mkdocs.yaml`: MkDocs Material, navigation, versioning, and i18n configuration.
- `translate_docs.py`: Helper script for translating Japanese Markdown into English.
- `requirements.txt`: Python dependencies for building and publishing the documentation.

Use `docs/ja` as the source language when adding or restructuring user-guide content.
Keep `README.md` files as local maintainer notes; they are not translation targets.

## Setup

Use Python 3 on Ubuntu 24.04 LTS or a compatible environment.

```bash
$ sudo apt install -y python3-dev python3-venv
$ python -m venv .venv
$ source .venv/bin/activate
$ pip install --upgrade pip
$ pip install -r requirements.txt
```

## Preview Locally

```bash
$ mkdocs serve --livereload
```

Then open http://127.0.0.1:8000/ in your browser.

## Translate Japanese to English

The translation helper uses the OpenAI API.

```bash
$ export OPENAI_API_KEY="your_api_key_here"
$ python translate_docs.py --help
```

Translate only changed Japanese Markdown files when updating existing English pages:

Choose a model and one of its supported reasoning levels from the
[OpenAI models documentation](https://developers.openai.com/api/docs/models),
then specify them with `--model` and `--reasoning-effort`.

```bash
$ python translate_docs.py --changed-only --src docs/ja --dst docs/en --base-ref HEAD --model gpt-5.6-sol --reasoning-effort max --dry-run
```

Review the generated diff, then run the same command without `--dry-run` to update the files.
The helper rejects translations that change protected code, links, URLs, HTML, or Markdown structure.
If an existing English file cannot be aligned safely with its Japanese source, inspect the files first and
use `--allow-full-fallback` only when replacing the entire English file is intended.
Deleted Japanese files are reported as warnings so that their English counterparts can be reviewed before removal.

## Deploy

Documentation versions are managed with `mike`.
See the [Material for MkDocs versioning guide](https://squidfunk.github.io/mkdocs-material/setup/setting-up-versioning/#usage)
for background.

```bash
$ mike deploy --push --update-aliases vx.x latest
$ mike set-default --push latest
```

To delete a specific version:

```bash
$ mike list
$ mike delete vx.x --push
```
