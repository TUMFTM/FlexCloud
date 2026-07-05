.PHONY: help
help:
	@echo "make helpgen|mkdocs|all|deploy"

.PHONY: helpgen
helpgen:
	@echo "Capturing CLI help output for :latest..."
	./scripts/capture_version.sh latest

.PHONY: mkdocs
mkdocs:
	@echo "Building documentation with Zensical..."
	cd .. && zensical build

.PHONY: all
all: mkdocs
	@echo "All documentation built."

.PHONY: deploy
deploy:
	@echo "Deploying documentation..."
	cd .. && zensical build
	cd .. && ghp-import --no-jekyll --push --force --message "Deploy documentation" site
