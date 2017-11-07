<p align="center">
  <img src="https://downloads.flytbase.com/flytwebsite/2017/04/Flyt-base-logo-1.png" alt="Flytbase: Get Access to Drone APIs and SDK" width="226">
  <br>
  <!-- <a href="https://travis-ci.org/lord/slate"><img src="https://travis-ci.org/lord/slate.svg?branch=master" alt="Build Status"></a> -->
</p>

<p align="center"><a href="api.flytbase.com">Documentation</a> for Flytbase API.</p>

<p align="center">This documentation was created with Slate. Check it out <a href="https://lord.github.io/slate">here</a>.</p>

Making Changes
------------------------------

### Prerequisites

You're going to need:

 - **Linux or OS X** — Windows may work, but is unsupported.
 - **Ruby, version 2.2.5 or newer**
 - **Bundler** — If Ruby is already installed, but the `bundle` command doesn't work, just run `gem install bundler` in a terminal.

### Getting Set Up

1. Fork this repository on Github.
2. Clone *your forked repository* (not our original one) to your hard drive with `git clone https://github.com/YOURUSERNAME/apidoc.git`
3. `cd apidoc`
4. Initialize and start local documentation server. This can be done locally, or with Vagrant:

```shell
# either run this to run locally
bundle install
bundle exec middleman server

# OR run this to run with vagrant
vagrant up
```
You can now see the docs at http://localhost:4567.

Now that the docs are all set up on your machine, you'll probably want to learn more about [editing Slate markdown](https://github.com/lord/slate/wiki/Markdown-Syntax).
If you'd prefer to use Docker, instructions are available [in the wiki](https://github.com/lord/slate/wiki/Docker).

Need Help? Found a bug?
--------------------

<!-- * [Submit an issue](https://github.com/flytbase/apidoc/issues) to the Flytbase API Doc Github if you need any help. -->

* Feel free to submit pull requests with bug fixes or changes.

* As a rule of thumb, it is suggested that small and granular changes be made in each pull request to keep the review and merge overhead small.
