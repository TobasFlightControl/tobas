# Tobas Documentation

## Setup (Ubuntu 24.04 LTS)

```bash
$ python -m venv .venv
$ source .venv/bin/activate
$ pip install --upgrade pip
$ pip install -r requirements.txt
```

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

## Local Test

1. Start the MkDocs server.

```bash
$ mkdocs serve --livereload
```

2. Then open http://127.0.0.1:8000/ in your browser.

## Deploy

[Material for MkDocs/Setting up versioning/Usage](https://squidfunk.github.io/mkdocs-material/setup/setting-up-versioning/#usage)

```bash
$ mike deploy --push --update-aliases vx.x latest
$ mike set-default --push latest
```

If you want to delete a specific version, execute:

```bash
$ mike list
$ mike delete vx.x --push
```
