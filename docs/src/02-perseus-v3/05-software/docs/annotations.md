# Annotations

To embed explanations which do not need their own page into your text use annotations.

## Usage

There are three parts to the annotation syntax:

- The location marker which is a number in brackets that specifies where the annotation symbol should appear
- The annotation class (`{ .annotate }` placed on the next line) which must be applied to a block to ensure the annotation is rendered
- The annotation content which is the next ordered list, with list number corresponding to marker numbers

All three of these must be met for the annotation to render properly

### Example

```txt linenums="1"
Lorem ipsum dolor sit amet, (1) consectetur adipiscing elit.
{ .annotate }

1. :man_raising_hand: I'm an annotation! I can contain `code`,
   **formatted text**, images, ... basically anything that can
   be expressed in Markdown.
```

Lorem ipsum dolor sit amet, (1) consectetur adipiscing elit.
{ .annotate }

1. :man_raising_hand: I'm an annotation! I can contain `code`,
   **formatted text**, images, ... basically anything that can
   be expressed in Markdown.
