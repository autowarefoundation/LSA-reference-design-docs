FROM python:3.11-slim

WORKDIR /app

RUN apt-get update && apt-get install -y curl && \
    curl -fsSL https://raw.githubusercontent.com/autowarefoundation/autoware-github-actions/main/deploy-docs/mkdocs-requirements.txt -o requirements.txt && \
    python3 -m pip install -U -r requirements.txt && \
    rm requirements.txt && \
    apt-get remove -y curl && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*

EXPOSE 8000

CMD ["mkdocs", "serve", "--dev-addr=0.0.0.0:8000"]
