[![Accelerate your robotics development](https://user-images.githubusercontent.com/14011012/195918769-5aaeedf3-5de2-48fb-951e-7399f2b9e190.png)](https://foxglove.dev)

<br/>

<div align="center">
    <h1>Foxglove Studio</h1>
    <a href="https://github.com/foxglove/studio/releases"><img src="https://img.shields.io/github/v/release/foxglove/studio?label=version" /></a>
    <a href="https://github.com/foxglove/studio/blob/main/LICENSE"><img src="https://img.shields.io/github/license/foxglove/studio" /></a>
    <a href="https://github.com/orgs/foxglove/discussions"><img src="https://img.shields.io/github/discussions/foxglove/community.svg?logo=github" /></a>
    <a href="https://foxglove.dev/slack"><img src="https://img.shields.io/badge/chat-slack-purple.svg?logo=slack" /></a>
    <br />
    <br />
    <a href="https://foxglove.dev/download">Download</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://docs.foxglove.dev/docs">Docs</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://foxglove.dev/blog">Blog</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://foxglove.dev/slack">Slack</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://twitter.com/foxglovedev">Twitter</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://foxglove.dev/contact">Contact Us</a>
  <br />
  <br />

[Foxglove](https://foxglove.dev) is an integrated visualization and diagnosis tool for robotics.

  <p align="center">
    <a href="https://foxglove.dev"><img alt="Foxglove Studio screenshot" src="/resources/screenshot.png"></a>
  </p>
</div>

<hr />

To learn more, visit the following resources:

[About](https://foxglove.dev/about)
&nbsp;•&nbsp;
[Documentation](https://docs.foxglove.dev/docs)
&nbsp;•&nbsp;
[Release notes](https://github.com/foxglove/studio/releases)
&nbsp;•&nbsp;
[Blog](https://foxglove.dev/blog)

You can join us on the following platforms to ask questions, share feedback, and stay up to date on what our team is working on:

[GitHub Discussions](https://github.com/orgs/foxglove/discussions)
&nbsp;•&nbsp;
[Slack](https://foxglove.dev/slack)
&nbsp;•&nbsp;
[Newsletter](https://foxglove.dev/#footer)
&nbsp;•&nbsp;
[Twitter](https://twitter.com/foxglovedev)
&nbsp;•&nbsp;
[LinkedIn](https://www.linkedin.com/company/foxglovedev/)

<br />

## Installation

Foxglove Studio is available online at [studio.foxglove.dev](https://studio.foxglove.dev/), or desktop releases can be downloaded from [foxglove.dev/download](https://foxglove.dev/download).

## Open Source

Foxglove Studio follows an open core licensing model. Most functionality is available in this repository, and can be reproduced or modified per the terms of the [Mozilla Public License v2.0](/LICENSE).

The official binary distributions available at [studio.foxglove.dev](https://studio.foxglove.dev/) or [foxglove.dev/download](https://foxglove.dev/download) incorporate some closed-source functionality, such as integration with [Foxglove Data Platform](https://foxglove.dev/data-platform), multiple layouts, private extensions, and more. For more information on free and paid features, see our [Pricing](https://foxglove.dev/pricing).

## Self-hosting

Foxglove Studio can be self-hosted using our [docker image](https://ghcr.io/foxglove/studio). Please note that this build does not contain any closed source functionality.

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
docker run --rm -p "8080:8080" -v /path/to/custom_layout.json:/foxglove/default-layout.json ghcr.io/foxglove/studio:latest
```

## Contributing

Foxglove Studio is written in TypeScript – contributions are welcome!

Note: All contributors must agree to our [Contributor License Agreement](https://github.com/foxglove/cla). See [CONTRIBUTING.md](CONTRIBUTING.md) for more details.

## Credits

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
