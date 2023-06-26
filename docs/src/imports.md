# Imports

The code of the trees can be organized as a project, breaking down the tree definitions into different files.
It enables the project to be organized logically avoiding redundancy. 

Therefore, the imports can appear in the file anywhere but are mostly grouped at the top, forming a sort of header.

## Syntax

### The whole file
To import the whole file, the following syntax needs to be applied:
```f-tree
import "nested/impls.tree"
import "/usr/home/projects/impls.tree"
import "C:\projects\forester\tree\tests\plain_project\nested\impls.tree"
```
### The definition with alias
```f-tree
import "nested/impls.tree" {
    grasp => grasp_ball,
}
```

## Import path
The path of the imports can be:
 - absolute : `C:\plain_project\nested\impls.tree`
 - relative : `nested/impls.tree`

### Absolute path
 ```f-tree
 import "C:\projects\forester\tree\tests\plain_project\nested\impls.tree"
 ```

### Relative path
 The relative path relates to the root of the project, that is pointed out in the start.
 Typically, the structure is the following:
 ```file
 - project_folder
    - main.tree // the file that has a root tree
    - .. folders
    - folder
        - def.tree
    - folder_nested
        - another_nested
            - file.tree    
```
Here the import of the `file.tree` can be
```
import "folder_nested/another_nested/file.tree"
```
in any other file.

## Aliases
To avoid the problem of ambiguous names when several tree definitions with the same name can be imported,
the aliases can come to the rescue.

They allow renaming tree definition while imports:

```f-tree
import "/robot_specific_ops/cv.tree" // has a tree def cv
import "/common_ops/cv.tree" { // also has a tree def cv 
    cv => com_cv // to avoid ambiguity, we can rename it using an alias.
}
```