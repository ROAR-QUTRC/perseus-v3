# Diagrams

Diagrams are excellent for representing complex system and should be used wherever possible. THe MkDocs plugin [mkdocs-drawio-exporter](https://github.com/LukeCarrier/mkdocs-drawio-exporter) is used here to dynamically export raw `.drawio` files to SVG's and embed them in pages using the same syntax as images in markdown.

# Usage

Place your `.drawio` file as close to the markdown file it belongs to as possible, alternatively shared diagrams should go into `docs/assets/drawio` (this is also where the exported SVG files live). It is recommended that you use the [Draw.io Integration](https://marketplace.visualstudio.com/items?itemName=hediet.vscode-drawio) VS Code plugin.

Then to include the digram use the markdown image syntax. For example, `! [my diagram](example.drawio)`[^1] (but without a space after the `!`, see below) where `my diagram` is the associated alt text and `example.drawio` is the diagram file that is in the same directory as this markdown file. The result is the following:

![my diagram](example.drawio)

!!! tip

    If your digram isn't updating try deleting the automatically generated `docs/assets/drawio/.cache` folder.

[^1]: The [mkdocs-drawio-exporter](https://github.com/LukeCarrier/mkdocs-drawio-exporter) plugin does not respect code blocks and will transform the contents if it matches the diagram syntax. The extra space is to break the syntax so that it isn't transformed.
