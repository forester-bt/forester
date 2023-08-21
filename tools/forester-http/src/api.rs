pub struct ForesterHttpApi {
    pub base: String,
}

impl ForesterHttpApi {
    pub fn trace_event(&self) -> String {
        format!("{}/tracer/custom", self.base)
    }
    pub fn print_trace(&self) -> String {
        format!("{}/tracer/print", self.base)
    }
    pub fn lock(&self, key: String) -> String {
        format!("{}/bb/{key}/lock", self.base)
    }
    pub fn unlock(&self, key: String) -> String {
        format!("{}/bb/{key}/unlock", self.base)
    }
    pub fn locked(&self, key: String) -> String {
        format!("{}/bb/{key}/locked", self.base)
    }
    pub fn contains(&self, key: String) -> String {
        format!("{}/bb/{key}/contains", self.base)
    }
    pub fn take(&self, key: String) -> String {
        format!("{}/bb/{key}/take", self.base)
    }
    pub fn get(&self, key: String) -> String {
        format!("{}/bb/{key}", self.base)
    }
    pub fn put(&self, key: String) -> String {
        format!("{}/bb/{key}", self.base)
    }
    pub fn new(base: String) -> Self {
        Self { base }
    }
}
