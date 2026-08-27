# Developing

## Motivation

This documentation is designed to be a guide to our open source rover projects. It has been separated from the main repoitory as it contains information about our team, past rovers and their iterations. These docs are contained in their own [repo](https://github.com/ROAR-QUTRC/roar-docs) and managed with [devenv.sh](https://devenv.sh) for a self contained reproducible environment.

## Maintenance

### Material for MkDocs is Abandoned

**Project Dependencies**

Minimal maintenance in the ways of dependency management is required as [Material for MkDocs](https://squidfunk.github.io/mkdocs-material/) is abandoned and will only be recieving security updates and major bug fixes. The Material for MkDocs dev team has moved onto a new project called [Zensical](https://zensical.org) which, when out of beta, should be a very easy direct replacement to Material for MkDocs.

**GitHub Actions**

The GitHub action workflow is based on two example workflows:

- The example for deploying a static website to github pages which can be found in the last section of this [article from GitHub](https://docs.github.com/en/pages/getting-started-with-github-pages/using-custom-workflows-with-github-pages#linking-separate-build-and-deploy-jobs)
- The [devenv.sh](https://devenv.sh/integrations/github-actions/) example for automatically building projects

MkDocs does have the command `mkdocs gh-deploy` which has not been used as it does not have much documentation and is not trusted primarily due to Material for MkDocs being abandoned.

!!! abstract "Last update"

    The actions used in the documentation workflow are up to date as of **07/10/26**

## Development Instructions

This project is managed by [devenv.sh](https://devenv.sh), which must be installed prior to running the project.

### Installing required tools

| Packet Manager | Steps                                                                                                                                                                                                                                      |
| -------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| apt            | Install `git` using: `sudo apt update && sudo apt install git`. <br /> Check the [devenv getting started page](https://devenv.sh/getting-started/) to find the appropriate installation method for your system (all of which require Nix). |
| Nix            | You need the packages `git` and `devenv`. Add them to your system config for NixOS or enter a shell with them using `nix-shell -p git devenv`.                                                                                             |

### Development Steps

Assuming [devenv.sh](https://devenv.sh) and `git` are installed development should typically follow the steps (Assuming you are in your home directoy):

**Setup**

1. Clone the repository with `git clone https://github.com/ROAR-QUTRC/roar-docs.git`
2. `cd` into the cloned repo: `cd ~/roar-docs`
3. If this is your first time then you will need to allow devenv by running `devenv allow`
4. Finally, if devenv does not automatically start cd out `cd ..` and back into the repo `cd roar-docs` and you should see devenv building the environment

??? warning "Not sure if it worked?"

    When you change directory you should see an output similar to this one:
    ```
    ✓ Validating lock                                                17ms
    ✓ Configuring shell                                              12ms
      └ ✓ Configuring cachix                                          2ms
      └ ✓ Evaluating shell cached                                     6ms
    ✓ Loading tasks                                                   1ms
      └ ✓ Evaluating devenv.config.task.config cached                 0ms
    ✓ Running tasks                                                  21ms
      └ ✓ devenv:enterShell                                           9ms
        └ ✓ devenv:files:cleanup                                      9ms
      └ ✓ devenv:enterTest skipped                                    0ms
    ```
    If you don't see this then go back and check you have installed [devenv.sh](https://devenv.sh) and try restarting your shell before seeking help from another team member.

**Development**

1. `cd` into the repository and wait for the devenv shell to load
2. Open the repository in your favourite IDE ([VS Code](https://code.visualstudio.com) is recommended)
3. Start the development server by running the command `dev` in the devenv shell and view the locally hosted website at [`http://127.0.0.1:8000`](http://127.0.0.1:8000)
4. Create or edit content following the guides in [Adding Content](./adding-content.md)
5. Once you make a new file or save changes you should be able to see live updates
