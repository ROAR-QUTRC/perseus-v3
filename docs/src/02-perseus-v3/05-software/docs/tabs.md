# Content Tabs

This feature allows you to compress content into horizontally stacked tabs. This can be very useful for showing code implemented in multiple languages.

## Usage

Start a tab using `=== "name"` where `name` must be replaced by the label of the tab. All the content for a tab should be below this line and indented.

For example this:

```
=== "First tab"
    First example
=== "Second tab"
    Alternate example
```

Will produce these tabs:

=== "First tab"

    First example

=== "Second tab"

    Alternate example

!!! tip "Tabs are linked"

    All tab groups on the same page with the same set of titles will be linked. For example by changing the example above this tab group should also update:

    === "First tab"

        First example

    === "Second tab"

        Alternate example
