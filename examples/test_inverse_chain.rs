use k::prelude::*;
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};

fn main() {
    // Create a simple 3-joint chain
    let fixed: k::Node<f32> = k::NodeBuilder::new()
        .name("fixed")
        .joint_type(k::JointType::Fixed)
        .translation(Translation3::new(0.0, 0.0, 0.6))
        .finalize()
        .into();
    let l0: k::Node<f32> = k::NodeBuilder::new()
        .name("joint0")
        .joint_type(k::JointType::Rotational {
            axis: Vector3::y_axis(),
        })
        .translation(Translation3::new(0.0, 0.1, 0.0))
        .finalize()
        .into();
    let l1: k::Node<f32> = k::NodeBuilder::new()
        .name("joint1")
        .joint_type(k::JointType::Rotational {
            axis: Vector3::x_axis(),
        })
        .translation(Translation3::new(0.0, 0.1, 0.0))
        .finalize()
        .into();
    let l2: k::Node<f32> = k::NodeBuilder::new()
        .name("end")
        .joint_type(k::JointType::Fixed)
        .translation(Translation3::new(0.0, 0.0, -0.30))
        .finalize()
        .into();
    
    k::connect![fixed => l0 => l1 => l2];
    
    let arm = k::SerialChain::new_unchecked(k::Chain::from_root(fixed));
    
    // Set joint angles
    let angles = vec![0.2, 0.3];
    arm.set_joint_positions(&angles).unwrap();
    arm.update_transforms();
    
    // Print world transforms
    println!("Original chain:");
    for node in arm.iter() {
        println!("  {} -> {:?}", node.joint().name, node.world_transform().unwrap().translation);
    }
    
    // Create inverse chain
    let arm_nodes: Vec<_> = arm.iter().collect();
    let world_transforms: Vec<_> = arm_nodes
        .iter()
        .map(|node| node.world_transform().unwrap())
        .collect();
    
    let mut i_nodes = Vec::new();
    
    for i in (0..arm_nodes.len()).rev() {
        let original_node = &arm_nodes[i];
        let new_node = k::Node::new(original_node.joint().clone());
        new_node.set_link(original_node.link().clone());
        
        if i == arm_nodes.len() - 1 {
            new_node.set_origin(world_transforms[0].clone());
        } else {
            let parent_world = &world_transforms[i + 1];
            let self_world = &world_transforms[i];
            let relative_transform = parent_world.inverse() * self_world;
            new_node.set_origin(relative_transform);
        }
        
        i_nodes.push(new_node);
    }
    
    for i in 1..i_nodes.len() {
        i_nodes[i].set_parent(&i_nodes[i - 1]);
    }
    
    let arm_i = k::SerialChain::new_unchecked(k::Chain::from_nodes(i_nodes));
    
    // Set reversed joint angles
    let angles_i: Vec<_> = angles.iter().rev().map(|x| *x).collect();
    arm_i.set_joint_positions(&angles_i).unwrap();
    arm_i.update_transforms();
    
    println!("\nInverse chain:");
    for node in arm_i.iter() {
        println!("  {} -> {:?}", node.joint().name, node.world_transform().unwrap().translation);
    }
    
    println!("\nChecking if nodes with the same name have the same position:");
    for arm_node in arm.iter() {
        let name = arm_node.joint().name.clone();
        if let Some(arm_i_node) = arm_i.find(&name) {
            let arm_pos = arm_node.world_transform().unwrap().translation;
            let arm_i_pos = arm_i_node.world_transform().unwrap().translation;
            let diff = (arm_pos.vector - arm_i_pos.vector).norm();
            println!("  {} - diff: {}", name, diff);
            if diff > 0.001 {
                println!("    WARNING: Position mismatch!");
                println!("    arm: {:?}", arm_pos);
                println!("    arm_i: {:?}", arm_i_pos);
            }
        }
    }
}