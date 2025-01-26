<h1 align="center">Lichtblick</h1>

<div align="center">
  <a href="https://github.com/lichtblick-suite/lichtblick/stargazers"><img src="https://img.shields.io/github/stars/lichtblick-suite/lichtblick" alt="Stars Badge"/></a>
  <a href="https://github.com/lichtblick-suite/lichtblick/network/members"><img src="https://img.shields.io/github/forks/lichtblick-suite/lichtblick" alt="Forks Badge"/></a>
  <a href="https://github.com/lichtblick-suite/lichtblick/pulls"><img src="https://img.shields.io/github/issues-pr/lichtblick-suite/lichtblick" alt="Pull Requests Badge"/></a>
  <a href="https://github.com/lichtblick-suite/lichtblick/issues"><img src="https://img.shields.io/github/issues/lichtblick-suite/lichtblick" alt="Issues Badge"/></a>
  <a href="https://github.com/lichtblick-suite/lichtblick/issues"><img src="https://img.shields.io/github/package-json/v/lichtblick-suite/lichtblick" alt="Versions Badge"/></a>
  <a href="https://github.com/lichtblick-suite/lichtblick/graphs/contributors"><img alt="GitHub contributors" src="https://img.shields.io/github/contributors/lichtblick-suite/lichtblick?color=2b9348"></a>
  <a href="https://opensource.org/licenses/MPL-2.0"><img src="https://img.shields.io/badge/License-MPL_2.0-brightgreen.svg" alt="License: MPL 2.0"></a>

  <br />
<p  align="center">
Lichtblick is an integrated visualization and diagnosis tool for robotics, available in your browser or as a desktop app on Linux, Windows, and macOS.
</p>
  <p align="center">
    <img alt="Lichtblick screenshot" src="resources/screenshot.png">
  </p>
</div>

**Dependencies:**

- [Node.js](https://nodejs.org/en/) v16.10+
- [Git LFS](https://git-lfs.github.com/)

<hr/>

## :rocket: Getting started

Clone the repository:

### Building the container

Please after cloning the repo you need to download some stuff from git lfs with:

```sh
git lfs pull
```

After that just build the container with:

```sh
docker build -t local-foxglove .

```

### Running the container

```sh
docker run --rm -p "8080:8080" local-foxglove
```

Foxglove Studio will be accessible in your browser at [localhost:8080](http://localhost:8080/).

### Developing with the container

The build process takes some minutes, so it is not practical to rebuild the container every time you make a change. You can use the following command to mount the local source code into the container and run the development server. But first, you need to modify the Dockerfile to comment the release part. For that, comment ALL lines from line 7 to the end of the Dockerfile. Then rebuild. After that, you can run the following command:

```sh
docker run --rm -it -p "8080:8080" --volume "${PWD}":"/src" --entrypoint bash local-foxglove
```

It should open a bash terminal inside the container. Then you can run the following commands to start the development server:

```sh
yarn install
yarn run web:serve
```

It will server the web app and recompile interactively when you make changes. You can access the web app at [localhost:8080](http://localhost:8080/).

### Overriding the default layout

[Bind-mount](https://docs.docker.com/storage/bind-mounts/) a layout JSON file at `/foxglove/default-layout.json` to set the default layout used when loading Studio from the Docker image.

```sh
$ git lfs pull
```

Enable corepack:

```sh
$ corepack enable
```

Install packages from `package.json`:

```sh
$ yarn install
```

Foxglove Studio originally began as a fork of [Webviz](https://github.com/cruise-automation/webviz), an open source project developed by [Cruise](https://getcruise.com/). Most of the Webviz code has been rewritten, but some files still carry a Cruise license header where appropriate.

## Using with Rosboard

We developed a new player compatible with also our own [fork of rosboard](https://github.com/kiwicampus/rosboard). To use it, you need to follow the instructions below:

1. **Use this Foxglove Fork**:
   You will need to use the [KiwiCampus fork for this project](https://github.com/kiwicampus/studio) or a derived version.

1. **Use our Rosboard Fork**:
   You will need to use the [KiwiCampus fork for Rosboard](https://github.com/kiwicampus/rosboard) on the robot you want to connect to.

1. **Connecting to Rosboard**:
   Similar to other available protocols (such as Rosbridge and Foxglove websocket), you can connect to Rosboard by following these steps:

   - Open the application.
   - Navigate to `File` -> `Open connection...`.
   - Select `Rosboard` from the list of available protocols.
   - Paste your websocket URL running Rosboard. Remeber it takes the form `ws://<ip>:<port>/rosboard/v1`.
   - Click `Open`.

- **Note**: Ensure that your Rosboard instance is running and accessible via the websocket URL you intend to use.
