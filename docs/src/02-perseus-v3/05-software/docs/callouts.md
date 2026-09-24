# Callouts

This feature should be used when basic text emphasis like bold, highlighting, and italics just isn't going to cut it. Below is a summary of how to use them but for further information including configuration visit the [docs](https://squidfunk.github.io/mkdocs-material/reference/admonitions/).

## Usage

### Basic

The minimum to create a call out is `!!!` followed by the callout variant name. A list of supported variants can seen below. However, any single word can be used as the variant and the colour theme will default to note. The content is optional and must be indented if specified. Here is an example:

```
!!! tip
    Remember to do the thing!
```

!!! tip

    Remember to do the thing!

### Titles

The title of a callout can be set by adding some text in quotes after the variant name:

```
!!! tip "Reminder"
    Remember to do the thing!
```

!!! tip "Reminder"

    Remember to do the thing!

The whole title section can be remove by using empty quotes `""`:

!!! tip ""

    Remember to do the thing!

### Collapsible Callouts

By replacing the `!!!` with `???` (`???+` for expanded by default) the callout will then become collapsible:

```
??? tip
    Remember to do the thing!
```

??? tip

    Remember to do the thing!

### Variants

!!! note

!!! abstract

!!! info

!!! tip

!!! success

!!! question

!!! warning

!!! failure

!!! danger

!!! bug

!!! example

!!! quote
