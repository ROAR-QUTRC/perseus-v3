# Adding Content

Adding content is as simple writing markdown (well mostly) through the magic of [Material for MkDocs](https://squidfunk.github.io/mkdocs-material/)! The markdown files inside the `docs` directory are rendered as pages using folders to define collapsible sections. Now of course, there are a few details to remember:

- The name of the file will be used as the section name unless a top level header `#` is specified
- To add content or change the name of collapsible sections place an `index.md` file inside the folder
- When the order matters prefix the file or folder with a zero padded 2 digit number (e.g. `03-name`)

All other extensions and features are well documented on the [Material for MkDocs](https://squidfunk.github.io/mkdocs-material/) website with a summary of useful features like [code blocks](code-blocks.md) and [call outs](callouts.md) in these docs.

## Markdown

A good reference if you've forgotten how to use markdown is this [cheat sheet](https://www.markdownguide.org/cheat-sheet/). All the following markdown features are supported:

- Headings
- Bold
- Italics
- Block quotes
- Ordered, unonrdered and task lists
- Code highlighting
- Links
- Images
- Tables
- Foot notes
- Sub and super text
- Term definitions
- Strike through
- Emojis
- Highlighting
- Math (using MathJax)

## Navigation Tree

Make sure when you are adding new docs that the titles and order of pages in the navigation section make sense!

The title of normal pages is set by adding a top level header to your file. Setting the title of expandable sections is slightly more involved. You must add an `index.md` file in the root of the directory you are naming and add the following to the top of the file:

```
---
title: <section title>
---
```

Replacing `<section title>` with the title you want to be displayed.

The order that files appear in the navigation tree is dictiated by the raw file names. To keep things consistent you must follow the number prefixing convention mentioned above and set custom titles for each section unless the order does not matter (such as the tutorials section).

!!! tip "Configuration location"

    Material for MkDocs does not support a nice way of specifying the page order. The [Awesome Nav for MkDocs](https://lukasgeiter.github.io/mkdocs-awesome-nav/) plugin fixes this. The configuration for this plugin lives in `docs/.nav.yml` file

??? info "Landing page config"

    The landing page must be in the root of the `/docs` directory but we want it to render in the Home tab. To achieve this a _very_ minimal navigation has been specified in `/docs/.nav.yml` to add the landing `index.md` to the Home tab.
