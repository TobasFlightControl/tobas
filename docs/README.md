# Tobas Documentation

## Setup (Ubuntu 24.04 LTS)

```bash
$ python -m venv .venv
$ source .venv/bin/activate
$ pip install --upgrade pip
$ pip install -r requirements.txt
```

## Test locally

1. Start the MkDocs server.

```bash
$ mkdocs serve --livereload
```

2. Then open http://127.0.0.1:8000/ in your browser.

## Translate Japanese to English

1. Install openai

```bash
$ pip install openai
```

2. Set OpenAI API key

```bash
$ export OPENAI_API_KEY="your_api_key_here"
```

3. Run the translation script

```bash
$ python translate_docs.py  # Try -h to see the available options.
```

> [!NOTE]
> The paths of local HTML links differ between the Japanese and English versions.

## Deploy Test

[Publishing your site](https://squidfunk.github.io/mkdocs-material/publishing-your-site/)

```bash
$ mkdocs build --clean
$ mkdocs gh-deploy --force --config-file mkdocs.yaml
```
