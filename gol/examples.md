![example1.jpg](example1.jpg)
```
root example1 {
  fallback {
    sequence {
        equal(battery_status, "low") // predef condition that takes 2 args and cmp them 
        drive_to(home)              // means drive_to(dest = home) where home is a key in bb
    }
    main_part(grasp_operation)
  }
}

sequence main_part (grasp_op:tree) {
  parallel {
    drive_to(kit_area)
    impl locate_kit                 // user action, it needs to be registered and implemented
  }
  grasp_op
  parallel calibrate {              // calibrate is just a label
    drive_to(placing_area)
    move_to(placing_pose)
  }
  parallel arm_op {
    impl place                      
    plan_arm_move(arm_home)
  }  
  move_arm_to(arm_home)        
}
sequence grasp_operation {
  move_arm_to(observation_pose)
  impl plan_grasping_pose
  parallel {
     impl grasp
     plan_arm_move(arm_home)
  }
  parallel {
     move_arm_to(arm_home)   
     impl plan_placing_pose
  }
}
impl drive_to(dest:object)
impl move_to(pose:object)
impl move_arm_to(pose:object)
impl plan_arm_move(dest:object)
```
---

![simple_ex.svg](simple_ex.svg)
```
root simple_ex sequence {
    handle_door
    impl enter_room
}

fallback handle_door {
    equal(door_state,"open")      
    open_door
    sequence {
        impl have_key
        impl unlock_door
        open_door
    }
    impl smash_door  
}

impl open_door

```
