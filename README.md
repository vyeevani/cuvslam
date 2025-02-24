## Directions

To use the cuVSLAM library, follow these steps:

1. Clone the repository:
    ```sh
    git clone <repository_url>
    cd <repository_directory>
    ```

2. Ensure you have the NVIDIA CUDA Toolkit installed. You can download it from [NVIDIA's official website](https://developer.nvidia.com/cuda-toolkit).

3. Set the `LD_LIBRARY_PATH` to point to the `lib` directory in the repository root:
    ```sh
    export LD_LIBRARY_PATH=$(pwd)/lib:$LD_LIBRARY_PATH
    ```

4. Build the project:
    ```sh
    cargo build
    ```

5. Run the tests to ensure everything is set up correctly:
    ```sh
    cargo test
    ```

Note: The tests have been run with the library shared path pointed to `repo_root/lib` and require the NVIDIA CUDA Toolkit to be installed on your system.
