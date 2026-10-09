# Minimal makefile for Sphinx documentation
#

ifeq ($(VERBOSE),1)
  Q =
else
  Q = @
endif

# You can set these variables from the command line.
SPHINXOPTS    ?=
SPHINXBUILD   = sphinx-build
SPHINXPROJ    = "EasyNavigation Documentation"
SOURCEDIR     = .
BUILDDIR      = _build

DOC_TAG      ?= development
RELEASE      ?= latest
PUBLISHDIR    = /tmp/EasyNav

# Put it first so that "make" without argument is like "make help".
help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)
	@echo ""
	@echo "make site"
	@echo "   build every version of versions.json in _site/ (the published website)"
	@echo "make publish"
	@echo "   build the site and push it to gh-pages (the GitHub Action does it on main)"

.PHONY: help Makefile site publish

# Generate the doxygen xml (for Sphinx) and copy the doxygen html to the
# api folder for publishing along with the Sphinx-generated API docs.

html:
	$(Q)$(SPHINXBUILD) -t $(DOC_TAG) -b html -d $(BUILDDIR)/doctrees $(SOURCEDIR) $(BUILDDIR)/html $(SPHINXOPTS) $(O)

# Every version of versions.json in _site/<version>/

site:
	$(Q)python3 scripts/build_site.py

# Remove generated content (Sphinx and doxygen)

clean:
	rm -fr $(BUILDDIR) _site

# Copy material over to the GitHub pages staging repo
# along with a README

publish: site
	rm -rf $(PUBLISHDIR)
	git clone --reference . https://github.com/EasyNavigation/EasyNavigation.github.io.git $(PUBLISHDIR)
	cd $(PUBLISHDIR) && \
	git checkout gh-pages && \
	git rm -rq . && git clean -fdq
	cp -r _site/. $(PUBLISHDIR)
	cd $(PUBLISHDIR) && \
	git add -A && \
	git diff-index --quiet HEAD || \
	(git commit -s -m "[skip ci] publish" && git push origin)
	rm -rf $(PUBLISHDIR)


# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
%: Makefile
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)
