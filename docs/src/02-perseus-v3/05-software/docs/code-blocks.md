# Code Blocks

You're probably farmilliar with the basic code blocks most mark down applications provide. There are a few handy features available that can make your code snippets much easier to read.

## Usage

### Syntax Highlighting

Write the [language shortcode](https://pygments.org/languages/) after the opening ` ``` ` for code blocks:

=== "Syntax"

    ````
    ```cpp
    #include <iostream>

    constexpr GREETING = "Hello, world!"

    int main()
    {
        std::cout << GREETING << std::endl;

        return 0;
    }
    ```
    ````

=== "Output"

    ```cpp
    #include <iostream>

    constexpr GREETING = "Hello, world!"

    int main()
    {
        std::cout << GREETING << std::endl;

        return 0;
    }
    ```

!!! tip "Inline snippets"

    For inline code snippets use the syntax `` `#!py console.log("Hello, world")` `` where `py` is the [language shortcode](https://pygments.org/languages/).

### Titles

Add `title="<custom title>"` to the opening line after the language name replacing `<custom title>` with the actual title:

=== "Syntax"

    ````
    ```cpp title="main.cpp"
    #include <iostream>

    constexpr GREETING = "Hello, world!"

    int main()
    {
        std::cout << GREETING << std::endl;

        return 0;
    }
    ```
    ````

=== "Output"

    ```cpp title="main.cpp"
    #include <iostream>

    constexpr GREETING = "Hello, world!"

    int main()
    {
        std::cout << GREETING << std::endl;

        return 0;
    }
    ```

### Line numbers

Line numbers can be added by using `linenums="<start>"` after the language name. The argument `<start>` is the number to start with on the first line.

=== "Syntax"

    ````
    ```cpp linenums="5"
    int main()
    {
        std::cout << GREETING << std::endl;

        return 0;
    }
    ```
    ````

=== "Output"

    ```cpp linenums="5"
    int main()
    {
        std::cout << GREETING << std::endl;

        return 0;
    }
    ```

### Highlighted Lines

Add `hl_lines="<lines>"` after the language name to enable this feature. The `<lines>` argument can be a list of individual numbers separated with spaces or a range separated by a dash or both. The line numbers used are independent of the `linenums` argument.

=== "Syntax"

    ````
    ```cpp hl_lines="1 3 5-7"
    #include <iostream>

    constexpr GREETING = "Hello, world!"

    int main()
    {
        std::cout << GREETING << std::endl;

        return 0;
    }
    ```
    ````

=== "Output"

    ```cpp hl_lines="1 3 7-9"
    #include <iostream>

    constexpr GREETING = "Hello, world!"

    int main()
    {
        std::cout << GREETING << std::endl;

        return 0;
    }
    ```

### Annotations

Annotations rely on comments from [syntax highlighting](#syntax-highlighting) and work similarly to [text annotation](./annotations.md). Place the annotation marker in a comment of the specified language then (optionally use a `!` to hide the extra comment characters) add your content in an ordered list after the code block.

=== "Syntax"

    ````

    ```cpp title="main.cpp"
    #include <iostream>

    constexpr GREETING = "Hello, world!"

    int main()
    {
        std::cout << GREETING << std::endl; // (1)!

        return 0;
    }
    ```

    1. You should really be using `#!cpp std::println` instead of `#!cpp std::cout`!
    ````

=== "Output"

    ```cpp title="main.cpp"
    #include <iostream>

    constexpr GREETING = "Hello, world!"

    int main()
    {
        std::cout << GREETING << std::endl; // (1)!

        return 0;
    }
    ```

    1. You should really be using `#!cpp std::println` instead of `#!cpp std::cout`!
