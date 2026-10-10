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
one (where `/` and the old unversioned URLs redirect). Each release is built from its maintenance
branch (`0.5.x` for `0.5.0`), created from its git tag; older releases (`0.4.2`) from the tag.

- `make site` builds every version in `_site/` (preview it with `cd _site && python3 -m http.server`).
- On every push to `main`, to a maintenance branch or a new tag, the GitHub Action `publish.yml`
  builds the site (always with the `versions.json` and scripts of `main`) and publishes it to
  `gh-pages`.
- New content goes only to `main` (`latest`). A maintenance branch only gets fixes of things that
  are wrong in the documentation of its release, by PR against it.

On a new release `<version>`: tag the website commit that documents it (`git tag <version>`),
create its maintenance branch (`git branch <version-prefix>.x <version>`, e.g. `0.6.x`), add
`{"name": "<version>", "ref": "origin/<version-prefix>.x", "title": "<version>"}` to
`versions.json`, add the branch to `on.push.branches` in `publish.yml`, set
`"default": "<version>"` and push.
