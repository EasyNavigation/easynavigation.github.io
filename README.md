# EasyNavigation.github.io

https://github.com/EasyNavigation/EasyNavigation.github.io

This folder holds the source and configuration files used to generate the
[EasyNav documentation](https://github.com/EasyNavigation/EasyNavigation.git) web site.

Dependencies for Build:

``` bash
sudo apt install python3-pip
pip3 install -r requirements.txt
```

Build the docs locally with `make html` and you'll find the built docs entry point in `_build/html/index.html`.


## Versions

The website has one version per EasyNav release, plus `latest` (development, built from `main`):
`https://easynavigation.github.io/<version>/`. They are listed in `versions.json`, with the default
one (where `/` and the old unversioned URLs redirect); each version is built from its git tag.

- `make site` builds every version in `_site/` (preview it with `cd _site && python3 -m http.server`).
- On every push to `main` or a new tag, the GitHub Action `publish.yml` builds the site and publishes
  it to `gh-pages`.

On a new release `<version>`: tag the website commit that documents it (`git tag <version>`), add
`{"name": "<version>", "ref": "<version>", "title": "<version>"}` to `versions.json`, set
`"default": "<version>"` and push.
