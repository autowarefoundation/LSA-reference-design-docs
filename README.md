# LSA-reference-design-docs

Repository to store mkdocs based documentation for the Reference Design Guideline for Low Speed Autonomy(LSA) vehicle.

You can access the document at [https://autowarefoundation.github.io/LSA-reference-design-docs/main/](https://autowarefoundation.github.io/LSA-reference-design-docs/main/)

## Test the document on localhost

* Prepare mkdocs container

```shell
make prepare
```

* Start mkdocs server on the built container

```shell
make serve
```

* You can access the local document on [http://127.0.0.1:8000/LSA-reference-design-docs/](http://127.0.0.1:8000/LSA-reference-design-docs/)

* Build static mkdocs documents

```shell
make build
```
