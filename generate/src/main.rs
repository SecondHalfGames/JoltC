mod api;
mod enums;

static PREFIX: &str = "JPC";

fn main() {
    let api = api::define_api();

    println!("==== HEADER ====");
    api.header(&mut std::io::stdout());

    println!("==== IMPL   ====");
    api.implementation(&mut std::io::stdout());

    // if let Err(err) = enums::generate() {
    //     eprintln!("Fatal error: {err:?}");
    // }
}
