![logo.png](pics%2Flogo.png)

# Forester
Forester represents a framework that provides the set of tools to perform the effective orchestration of the set of tasks.\
The tasks can be performed synchronously or asynchronously, locally or remotely.\
Forester takes care of the correct performance and distribution of the tasks.\
the main concept of the framework
is the flow based on the [behavior trees](https://en.wikipedia.org/wiki/Behavior_tree_(artificial_intelligence,_robotics_and_control)#:~:text=A%20behavior%20tree%20is%20a,tasks%20in%20a%20modular%20fashion.)\
it can be effectively used in the game, ai, robotic areas, or anywhere where the workflow engine can be applied.

## Why Forester
The main idea and the target of Forester is to make the process of chaining a complex logic \
of the different tasks together effective and easy.

The following set of features is summoned to highlight the framework among the others. 
 
#### The dsl to describe the logic
One of the problems that Forester endeavours to solve is to isolate the logic of the orchestration \ 
from the logic of the tasks implementations and therefore the dsl ('Tree') is provided. \
The Dsl is script based and supports a number of features that can alleviate the writing of the big trees.

#### The framework provides the ability to create async and sync tasks
The tasks (leaves of the tree) can be fulfilled with the asynchronous and synchronous logic.
The difference here is the async tasks will not block the tree while sync tasks will block the tree.

#### The framework provides the ability to create remote and local tasks (TBD)
The tasks can represent as a local stateless/stateful blocks of logic as the remote servers or procedures.

#### The tooling to visualize and trace the execution of the tree
The tree can be visualized and traced in order to see how it is supposed to be executed.

#### The simulation mode is supposed to aid with the design decisions
The special simulation mode aims to help, quickly to see how the tree will be unfolding and how it can be designed.

#### The optimizations and analysis of the tree (TBD)
The language provides a set of optimizations and validations to either ensure the logic is correct \
or perform some validations on it.

#### The validations engine allows the users to create the manually defined validations (TBD)
The user-defined validations can be useful to restrict some features of the framework.  

#### Integrations (TBD)

## Why behavior trees
Firstly, they provide a strong math abstraction over the orchestration logic \
and enables to separate the business logic and the tree logic itself.
One of the great advantages of the behavior trees is that they provide a good conception of modularity. \
On the other hand, they have only a small set of logically conjucted components making the design easier,

### Articles that introduce into the basics of the behaviour trees
- [Chris Simpson’s Behavior trees for AI: How they work](https://outforafight.wordpress.com/2014/07/15/behaviour-behavior-trees-for-ai-dudes-part-1/)
- [Introduction to behavior trees](https://robohub.org/introduction-to-behavior-trees/)
- [State Machines vs Behavior Trees](https://www.polymathrobotics.com/blog/state-machines-vs-behavior-trees)

### Useful libraries
- [BehaviorTree.CPP](https://www.behaviortree.dev/) : the brilliant library provides the implementation on CPP.
- [Beehave](https://github.com/bitbrain/beehave) :  behavior tree AI for Godot Engine.
- [Bonsai](https://github.com/Sollimann/bonsai) : the great library for behavior trees in rust. 

