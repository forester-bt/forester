# Tree language

The tree language is a frontend for the framework itself.
Generally, the language is a simple dsl encompassing the basic abstractions \
and enabling to create of the building block upon the abstractions \

### Why the language is needed

The basic idea behind the language is an attempt to provide a set of generalizations \
which will alleviate the redundancy in some cases.

- The language allows creating the tree definitions accepting other trees as parameters (higher order trees)
- The language provides lambda definitions

The syntax of the language is very simple and is described in this chapter.

### Structure of the project

The scripts are supposed to be in the folder which is marked as `root` directory.
All imports start from the root and represent a path relating to the root directory:

```file
 - project_folder
    - main.tree
    - gripper.tree
    - cv.tree
    - utility
        - utility.tree
        - helpers.tree
```

The project should have at least one `root` tree [definition](./definitions.md). If the project has several,
the one that is supposed to run needs to be pointed out to.

### File extension
The files have the extension `tree`.
