use forester_rs::runtime::forester::serv::ApiDoc;
use std::env;
use std::fs::File;
use std::io::BufWriter;
use std::path::PathBuf;
use utoipa::OpenApi;

fn main() {
    let output = env::args()
        .nth(1)
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("openapi.json"));

    let doc = ApiDoc::openapi();
    let file = File::create(&output)
        .unwrap_or_else(|e| panic!("cannot create {}: {e}", output.display()));
    let writer = BufWriter::new(file);
    serde_json::to_writer_pretty(writer, &doc)
        .unwrap_or_else(|e| panic!("cannot serialize the OpenAPI spec: {e}"));

    println!("OpenAPI spec written to {}", output.display());
}
