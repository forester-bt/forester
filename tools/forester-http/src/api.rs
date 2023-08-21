/// The api to Forester instance of http server
/// It is used to get access to the blackboard and to trace events.
///
/// #Example
/// ```rust
/// use forester_http::api::ForesterHttpApi;
///
/// async fn main() {
///     let api = ForesterHttpApi::new("http://localhost:10000".to_string());
///     let  _ = reqwest::get(api.print_trace()).await;
/// }
///
/// ```
pub struct ForesterHttpApi {
    /// The base url of the Forester instance. Typically it is http://localhost:10000
    pub base: String,
}

impl ForesterHttpApi {
    /// creates a new trace event
    pub fn trace_event(&self) -> String {
        format!("{}/tracer/custom", self.base)
    }
    /// prints the trace or if the file is big the tail of the trace (last 100 lines)
    pub fn print_trace(&self) -> String {
        format!("{}/tracer/print", self.base)
    }
    /// lock the key in the blackboard
    pub fn lock(&self, key: String) -> String {
        format!("{}/bb/{key}/lock", self.base)
    }
    /// unlock the key in the blackboard
    pub fn unlock(&self, key: String) -> String {
        format!("{}/bb/{key}/unlock", self.base)
    }
    /// check if the key is locked in the blackboard
    pub fn locked(&self, key: String) -> String {
        format!("{}/bb/{key}/locked", self.base)
    }
    /// check if the key is in the blackboard
    pub fn contains(&self, key: String) -> String {
        format!("{}/bb/{key}/contains", self.base)
    }
    /// take the key from the blackboard.
    pub fn take(&self, key: String) -> String {
        format!("{}/bb/{key}/take", self.base)
    }
    /// get the key from the blackboard.
    pub fn get(&self, key: String) -> String {
        format!("{}/bb/{key}", self.base)
    }
    /// put the key to the blackboard.
    pub fn put(&self, key: String) -> String {
        format!("{}/bb/{key}", self.base)
    }
    pub fn new(base: String) -> Self {
        Self { base }
    }
}
