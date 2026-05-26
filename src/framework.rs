pub const MAX_NODES: usize = 32;
pub const MAX_PORTS_PER_NODE: usize = 8;
pub const MAX_BINDINGS: usize = 64;

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum PortValue {
    Float(f32),
    Bool(bool),
    Int(i32),
}

impl PortValue {
    pub fn float_or(&self, fallback: f32) -> f32 {
        match self {
            Self::Float(v) => *v,
            _ => fallback,
        }
    }

    pub fn bool_or(&self, fallback: bool) -> bool {
        match self {
            Self::Bool(v) => *v,
            _ => fallback,
        }
    }

    pub fn int_or(&self, fallback: i32) -> i32 {
        match self {
            Self::Int(v) => *v,
            _ => fallback,
        }
    }
}

/// The safe, idiomatic Rust Trait governing our physical/virtual hardware logic
pub trait Node {
    fn update(&mut self, inputs: &[PortValue], outputs: &mut [PortValue], delta_time_ms: u32);
}

/// Helper container holding native node states and I/O buffers
pub struct NodeContainer {
    pub node_id: u32,
    pub inner: CoreNode,
    pub inputs: [PortValue; MAX_PORTS_PER_NODE],
    pub input_count: usize,
    pub outputs: [PortValue; MAX_PORTS_PER_NODE],
    pub output_count: usize,
}

/// High-performance static dispatch enum bypasses heap allocation for bare-metal targets,
/// while maintaining FFI compatibility via custom dynamic hooks.
pub enum CoreNode {
    Pid(crate::nodes::PidController),
    PhaseAngle(crate::nodes::PhaseAngleMapper),
    Sequencer(crate::nodes::Sequencer),
    Safety(crate::nodes::SafetyWatchdog),
    Thermostat(crate::nodes::Thermostat),
    VirtualPressure(crate::nodes::VirtualPressureSensor),
    VirtualFlow(crate::nodes::VirtualFlowSensor),
    Custom {
        update_fn: extern "C" fn(
            state: *mut core::ffi::c_void,
            inputs: *const crate::ffi::PortC,
            outputs: *mut crate::ffi::PortC,
            dt: u32,
        ),
        state: *mut core::ffi::c_void,
    },
}

impl CoreNode {
    pub fn update(&mut self, inputs: &[PortValue], outputs: &mut [PortValue], delta_time_ms: u32) {
        match self {
            Self::Pid(n) => n.update(inputs, outputs, delta_time_ms),
            Self::PhaseAngle(n) => n.update(inputs, outputs, delta_time_ms),
            Self::Sequencer(n) => n.update(inputs, outputs, delta_time_ms),
            Self::Safety(n) => n.update(inputs, outputs, delta_time_ms),
            Self::Thermostat(n) => n.update(inputs, outputs, delta_time_ms),
            Self::VirtualPressure(n) => n.update(inputs, outputs, delta_time_ms),
            Self::VirtualFlow(n) => n.update(inputs, outputs, delta_time_ms),
            Self::Custom { update_fn, state } => {
                // To support true no_std without heap allocation, we translate
                // PortValues to C-compatible structures on the local stack.
                let mut c_inputs = [crate::ffi::PortC {
                    value_type: crate::ffi::PortDataTypeC::Int,
                    value: crate::ffi::PortValueC { i_val: 0 },
                }; MAX_PORTS_PER_NODE];

                for (idx, val) in inputs.iter().enumerate() {
                    if idx < MAX_PORTS_PER_NODE {
                        c_inputs[idx] = crate::ffi::native_to_c(*val);
                    }
                }

                let mut c_outputs = [crate::ffi::PortC {
                    value_type: crate::ffi::PortDataTypeC::Int,
                    value: crate::ffi::PortValueC { i_val: 0 },
                }; MAX_PORTS_PER_NODE];

                for (idx, val) in outputs.iter().enumerate() {
                    if idx < MAX_PORTS_PER_NODE {
                        c_outputs[idx] = crate::ffi::native_to_c(*val);
                    }
                }

                // Call custom dynamic node safely across the FFI boundary using C-stable structures
                update_fn(
                    *state,
                    c_inputs.as_ptr(),
                    c_outputs.as_mut_ptr(),
                    delta_time_ms,
                );

                // Re-hydrate safe native Rust values from the callback's returned changes
                for (idx, val) in outputs.iter_mut().enumerate() {
                    if idx < MAX_PORTS_PER_NODE {
                        *val = crate::ffi::c_to_native(c_outputs[idx]);
                    }
                }
            }
        }
    }
}

#[derive(Copy, Clone)]
pub struct Binding {
    pub src_node_id: u32,
    pub src_port_idx: usize,
    pub dest_node_id: u32,
    pub dest_port_idx: usize,
}

pub struct SafeScheduler {
    pub nodes: [Option<NodeContainer>; MAX_NODES],
    pub bindings: [Option<Binding>; MAX_BINDINGS],
}

impl SafeScheduler {
    pub const fn new() -> Self {
        const EMPTY_NODE: Option<NodeContainer> = None;
        const EMPTY_BINDING: Option<Binding> = None;

        Self {
            nodes: [EMPTY_NODE; MAX_NODES],
            bindings: [EMPTY_BINDING; MAX_BINDINGS],
        }
    }

    pub fn add_node(
        &mut self,
        node_id: u32,
        inner: CoreNode,
        input_count: usize,
        output_count: usize,
    ) -> bool {
        for slot in self.nodes.iter_mut() {
            if slot.is_none() {
                *slot = Some(NodeContainer {
                    node_id,
                    inner,
                    inputs: [PortValue::Int(0); MAX_PORTS_PER_NODE],
                    input_count: input_count.min(MAX_PORTS_PER_NODE),
                    outputs: [PortValue::Int(0); MAX_PORTS_PER_NODE],
                    output_count: output_count.min(MAX_PORTS_PER_NODE),
                });
                return true;
            }
        }
        false
    }

    pub fn add_binding(
        &mut self,
        src_node: u32,
        src_port: usize,
        dest_node: u32,
        dest_port: usize,
    ) -> bool {
        for slot in self.bindings.iter_mut() {
            if slot.is_none() {
                *slot = Some(Binding {
                    src_node_id: src_node,
                    src_port_idx: src_port,
                    dest_node_id: dest_node,
                    dest_port_idx: dest_port,
                });
                return true;
            }
        }
        false
    }

    pub fn set_input(&mut self, node_id: u32, port_idx: usize, val: PortValue) -> bool {
        if let Some(node) = self.find_node_mut(node_id) {
            if port_idx < node.input_count {
                node.inputs[port_idx] = val;
                return true;
            }
        }
        false
    }

    pub fn get_output(&self, node_id: u32, port_idx: usize) -> Option<PortValue> {
        self.find_node(node_id).and_then(|node| {
            if port_idx < node.output_count {
                Some(node.outputs[port_idx])
            } else {
                None
            }
        })
    }

    pub fn propagate(&mut self) {
        for i in 0..MAX_BINDINGS {
            if let Some(binding) = self.bindings[i] {
                let mut propagated_val: Option<PortValue> = None;

                if let Some(src) = self.find_node(binding.src_node_id) {
                    if binding.src_port_idx < src.output_count {
                        propagated_val = Some(src.outputs[binding.src_port_idx]);
                    }
                }

                if let Some(val) = propagated_val {
                    if let Some(dest) = self.find_node_mut(binding.dest_node_id) {
                        if binding.dest_port_idx < dest.input_count {
                            dest.inputs[binding.dest_port_idx] = val;
                        }
                    }
                }
            }
        }
    }

    pub fn tick(&mut self, delta_time_ms: u32) {
        self.propagate();
        for i in 0..MAX_NODES {
            if let Some(ref mut container) = self.nodes[i] {
                container
                    .inner
                    .update(&container.inputs, &mut container.outputs, delta_time_ms);
            }
        }
    }

    fn find_node(&self, node_id: u32) -> Option<&NodeContainer> {
        for slot in self.nodes.iter() {
            if let Some(node) = slot {
                if node.node_id == node_id {
                    return Some(node);
                }
            }
        }
        None
    }

    fn find_node_mut(&mut self, node_id: u32) -> Option<&mut NodeContainer> {
        for slot in self.nodes.iter_mut() {
            if let Some(node) = slot {
                if node.node_id == node_id {
                    return Some(node);
                }
            }
        }
        None
    }
}
