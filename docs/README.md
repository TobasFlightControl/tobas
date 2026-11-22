# Tobas Documentation

This directory contains the MkDocs project used to build the Tobas user guide.
It is intended for people editing, translating, previewing, or publishing the documentation.
If you want to install or use Tobas, start from the published guide:

- https://tobasflightcontrol.github.io/tobas/latest/

## **Structure**

- `docs/ja`: Japanese source documentation.
- `docs/en`: English documentation generated or maintained from the Japanese source.
- `mkdocs.yaml`: MkDocs Material, navigation, versioning, and i18n configuration.
- `translate_docs.py`: Helper script for translating Japanese Markdown into English.
- `requirements.txt`: Python dependencies for building and publishing the documentation.

Use `docs/ja` as the source language when adding or restructuring user-guide content.
Keep `README.md` files as local maintainer notes; they are not translation targets.

## **Setup**

Use Python 3 on Ubuntu 24.04 LTS or a compatible environment.

```bash
$ python -m venv .venv
$ source .venv/bin/activate
$ pip install --upgrade pip
$ pip install -r requirements.txt
```

## **Preview Locally**

```bash
$ mkdocs serve --livereload
```

Then open http://127.0.0.1:8000/ in your browser.

## **Deploy**

Documentation versions are managed with `mike`.
See the [Material for MkDocs versioning guide](https://squidfunk.github.io/mkdocs-material/setup/setting-up-versioning/#usage)
for background.

```bash
$ mike deploy --push --update-aliases jazzy latest
$ mike set-default --push latest
```

To delete a specific version:

```bash
$ mike list
$ mike delete <target> --push
```
