PYTHON = python3
MAIN = main.py
MAP ?=

install:
	$(PYTHON) -m pip install --upgrade pip
	$(PYTHON) -m pip install -r requirements.txt
	$(PYTHON) -m pip install flake8 mypy pytest

run:
	$(PYTHON) $(MAIN) $(MAP)

debug:
	$(PYTHON) -m pdb $(MAIN) $(MAP)

clean:
	find . -type d -name "__pycache__" -not -path "./venv/*" -exec rm -rf {} +
	rm -rf .mypy_cache .pytest_cache

lint:
	flake8 . --exclude venv
	mypy . --warn-return-any --warn-unused-ignores --ignore-missing-imports --disallow-untyped-defs --check-untyped-defs --exclude venv

lint-strict:
	flake8 . --exclude venv
	mypy . --strict --exclude venv

.PHONY: install run debug clean lint lint-strict