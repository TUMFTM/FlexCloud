.PHONY: help
help:
	@echo "make helpgen|mkdocs|all|deploy"

.PHONY: helpgen
helpgen:
	@echo "Capturing CLI help output for :latest..."
	./scripts/capture_version.sh latest

.PHONY: mkdocs
mkdocs:
	@echo "Building MkDocs documentation..."
	cd .. && mkdocs build

.PHONY: all
all: mkdocs
	@echo "All documentation built."

.PHONY: deploy
deploy:
	@echo "Deploying documentation..."
	cd .. && mkdocs gh-deploy --force
