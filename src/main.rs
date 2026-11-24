mod io;
mod model;
mod results;
mod plotting;

use io::truss_file::read_truss;
use model::truss::Truss;
use std::path::Path;
use plotting::plot;

fn main() {
    let truss = read_truss(Path::new("./examples/truss.json")).unwrap_or_else(|error| {
        eprintln!("failed to read truss: {error}");
        std::process::exit(1);
    });

    let result = truss.solve_truss();

    let displacements = &result.displacements;
    let reactions = &result.reactions;
    println!("displacements: {displacements:#?}");
    println!("reactions: {reactions:#?}");

    let path = "./plots/version1.png";
    plot::undeformed_truss(&truss, path).unwrap();

    let path =  "./plots/deformed.png";
    plot::deformed_truss(&truss, &result, 100000.0, path).unwrap();


}


