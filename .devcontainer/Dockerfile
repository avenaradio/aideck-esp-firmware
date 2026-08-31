FROM bitcraze/builder

SHELL ["/bin/bash", "-c"]

# Make ESP-IDF 4.3 the default environment for every shell.
RUN echo 'source /new_home/esp/esp-idf/export.sh' >> /root/.bashrc

# Also make it available to non-interactive shells / VS Code tasks.
ENV IDF_PATH=/new_home/esp/esp-idf
ENV PATH=/new_home/esp/esp-idf/tools:$PATH

# Use the Python environment belonging to ESP-IDF 4.3.
ENV IDF_PYTHON_ENV_PATH=/new_home/.espressif/python_env/idf4.3_py3.10_env

ENV PATH=/new_home/.espressif/python_env/idf4.3_py3.10_env/bin:$PATH

# Fix the setuptools version required by the old IDF environment.
RUN source "$IDF_PYTHON_ENV_PATH/bin/activate" && \
    python -m pip install --force-reinstall --no-cache-dir "setuptools<81" && \
    python -m pip install --no-cache-dir click

WORKDIR /module

CMD ["/bin/bash"]
