# Visualization

The engine provides the ability to visualize a given project.

## Precausions

To get it worked locally, the [graphviz](https://graphviz.org/download/) should be installed, 
since the visualizations uses it under the hood. 

## Example

![amr_simple.svg](./pics/amr_simple.svg)

## How to use

### Console utility

```shell
forester vis --root project/ --main main.tree --tree main --output viz.svg
```

- root can be ommited, the `<pwd>` folder will be taken by default
- main can be ommited, by default, the name `main.tree` will be taken.  
- tree can be ommited if only one root definition in the file
- output can be ommited, by default, the name of the main file will be taken but the extention will be `svg`  
