# The console utility to work with [Forester](https://github.com/besok/forester)

The details can be found in the [book](https://besok.github.io/forester/api.html)

The commands:

```shell
Commands:
  sim   Runs simulation. Expects a simulation profile
  vis   Runs visualization. Output is in svg format.
  help  Print this message or the help of the given subcommand(s)

```

## Simulation (sim)

```shell
Options:
  -p, --profile <PATH>  a path to a sim profile
  -r, --root <ROOT>     a path to a root folder. The <PWD> folder by default
  -m, --main <MAIN>     a path to a main file. The 'main.tree' by default
  -t, --tree <TREE>     a root in a main file. If there is only one root it takes by default
  -h, --help            Print help
```

## Visualization

```shell
Options:
  -o, --output <OUTPUT>  a file for svg. If  no, the name from the main file will be taken.
  -r, --root <ROOT>      a path to a root folder. The <PWD> folder by default
  -m, --main <MAIN>      a path to a main file. The 'main.tree' by default
  -t, --tree <TREE>      a root in a main file. If there is only one root it takes by default
  -h, --help             Print help

```

