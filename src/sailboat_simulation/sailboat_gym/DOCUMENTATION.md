# Sailboat Gym Documentation

This documentation provides detailed information about the Sailboat Gym package, which includes the `SailboatLSAEnv` environment and the `CV2DRenderer` renderer. It covers various aspects of the package, including customization options, container tags, debugging/profiling, and provides usage examples.

## Table of Contents

- [Table of Contents](#table-of-contents)
- [Environment (`SailboatLSAEnv`)](#environment-sailboatlsaenv)
- [2D Renderer (`CV2DRenderer`)](#2d-renderer-cv2drenderer)
- [Container tags (`container_tag`)](#container-tags-container_tag)
- [Debugging/Profiling](#debuggingprofiling)
- [Examples](#examples)


## Environment (`SailboatLSAEnv`)

The `SailboatLSAEnv` environment in the Sailboat Gym package provides a flexible and customizable simulation environment for sailboat control tasks. You can customize several parameters to tailor the environment to your specific requirements. These parameters include:

- `reward_fn`: A callable function that computes the reward based on the current observation and action. You can define a custom reward function for your specific task.
- `renderer`: An instance of a renderer used for visualizing the environment. You can choose to use the provided `CV2DRenderer` or provide your own renderer implementation.
- `wind_generator_fn`: A function that generates a 2D vector representing the global wind during the simulation. You can use a custom wind generator function to simulate different wind conditions.
- `video_speed`: The playback speed of the recorded video. You can adjust the speed at which the simulation video is played back.
- `keep_sim_alive`: A boolean value that determines whether the Docker simulation will be kept alive after the simulation ends. Setting this parameter to `True` can be useful for debugging or speeding up the initialization of the simulation.
- `container_tag`: The container tag used for the simulation, which determines the maximum step size (mss) of the simulation. Please refer to the [available container tags section](#container-tags-container_tag) for more information.

Please refer to the [Observation](./README.md#observation-space) and [Action](./README.md#action-space) sections for detailed information about the observation and action spaces within the `SailboatLSAEnv` environment.

## 2D Renderer (`CV2DRenderer`)

The Sailboat Gym package includes a 2D renderer called `CV2DRenderer` that allows you to visualize the sailboat environment in a 2D representation. The `CV2DRenderer` provides customizable parameters to control the appearance and style of the rendered image. These parameters are:

- `size`: The size of the rendered image in pixels.
- `padding`: The amount of padding around the rendered image.
- `vector_scale`: The scale factor for rendering vectors.
- `style`: A dictionary of keyword arguments that can be used to modify the default rendering options. You can override specific rendering settings such as colors, widths, and sizes for different components of the rendered image using this parameter.

You can find additional information about the default rendering options in the [default rendering options](sailboat_gym/renderers/cv_2d_renderer.py) file.

## Container tags (`container_tag`)

The container tags in the Sailboat Gym package allow you to control the maximum step size (mss) of the simulation. The default tag, `mss1`, corresponds to a maximum step size of 1 ms, which aligns with the default gazebo step size. The available container tags and their corresponding step sizes are:

- `realtime`: Simulates in real-time without any speed-up.
- `mss1`: Maximum step size of 1 ms.
- `mss2`: Maximum step size of 2 ms.
- `mss3`: Maximum step size of 3 ms.
- `mss4`: Maximum step size of 4 ms.
- `mss5`: Maximum step size of 5 ms.
- `mss6`: Maximum step size of 6 ms.
- `mss7`: Maximum step size of 7 ms.
- `mss8`: Maximum step size of 8 ms.
- `mss9`: Maximum step size of 9 ms.
- `mss10`: Maximum step size of 10 ms.

Choosing a larger maximum step size allows the simulation to run faster but may result in decreased accuracy. When using the `realtime` tag, the simulation will utilize the `mss1` tag with an update rate of **1000 Hz**, matching the default gazebo update rate. For other tags, the simulation will use the **fastest possible update rate** by setting the gazebo's `real-time update rate` to **0**.

## Debugging/Profiling

The Sailboat Gym package provides support for debugging and profiling through the use of environment variables. The following environment variables are available:

- `DEBUG`: Setting this variable to `1` allows you to display debug information during the simulation. It can be useful for gaining insights into the internal workings and behaviors of the package.
- `PROFILING`: Setting this variable to `1` enables the display of profiling information during the simulation. This is particularly useful for benchmarking and performance analysis. Additionally, setting it to `all` provides detailed profiling information for all relevant functions.

To utilize these debugging and profiling features, set the corresponding environment variables before running the code.

## Examples

To help users understand the usage and behavior of the Sailboat Gym package, here are a few examples:

1. To enable debug information during the simulation, use the following command:

```bash
DEBUG=1 python3 example.py
```

This command sets the `DEBUG` environment variable to `1`, enabling the display of debug information during the simulation.

2. To display profiling information during the simulation, use the following command:

```bash
PROFILING=1 python3 example.py
```

This command sets the `PROFILING` environment variable to `1`, enabling the display of profiling information during the simulation.

The examples showcase the usage of the Sailboat Gym package in different scenarios and highlight the output and behavior under specific configurations. Please refer to the relevant sections for more detailed information about each topic and to ensure a comprehensive understanding of the Sailboat Gym package.