# CI/CD

## What is CI/CD?

CI/CD (which stands for Continuous Integration, Continuous Delivery/Deployment) is, in theory, exactly what it says on the tin.

### Continuous Integration

Continuous Integration refers to frequently (and automatically) uploading and merging code to the main repository.
This keeps code merges small (reducing the chances and sizes of merge conflicts), and therefore more manageable.
However, that's only one part of code _integration_ - the other is continuous and _automated_ testing.
CI/CD is only made possible by continuous and automated builds and tests which run every time code is pushed to the main repository.

### Continuous Delivery

Continuous Delivery refers to the project always being _delivered_ in a functional, ready-to-go state, and handles any final stages needed to package the project and get it into a _deployment_ ready state.
Since this project is built with Nix, all dependencies are fixed and known ahead of time, so there's nothing extra to do here.

### Continuous Deployment

Continuous Deployment is exactly what it sounds like - automatically _deploying_ a project to production after the continuous _delivery_ process of the pipeline finishes its build.
For this project, we aren't employing continuous deployment for anything but the docs website - see below for details on that.

## Execution

The CI/CD pipeline for this project is run entirely using [GitHub Actions](https://docs.github.com/en/actions).
The typical workflow looks something like this:

1. Check out the repo with [`actions/checkout`](https://github.com/actions/checkout)
2. Install Nix with [`cachix/install-nix-action`](https://github.com/cachix/install-nix-action/)
3. Set up Nix output caching with [`cachix/cachix-action`](https://github.com/cachix/cachix-action/)
4. Build the environment and run all tests using `devenv --profile cicd test`[^cicd-profile]
5. If that succeeds, the builds are passing!
6. Upload to Cachix so we can just download the builds next time

[^cicd-profile]: This profile inherits from all the others, so it builds everything.

If you're curious about any specific workflow, they're all well commented.

## Maintenance

Maintenance required involves ensuring that the actions used are up to date. A very common warning returned by actions is `Node XX is being deprecated.` which is commonly fixed by incrementing the version number of the action.

The `cicd` devenv profile must be update to extend new profiles to ensure that they are built and cached during CI/CD.
