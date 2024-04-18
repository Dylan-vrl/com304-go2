main:
	mkdir -p build && \
	cd build && \
	cmake .. && \
	make && \
	python3 setup.py bdist_wheel && \
	find dist -maxdepth 1 -type f -printf '%h/%f' | xargs $(VENV_PATH) -m pip install -U --force-reinstall