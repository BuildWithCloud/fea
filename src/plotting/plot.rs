use plotters::prelude::*;

use crate::{Truss, results::truss_result::TrussResult};

pub fn undeformed_truss(truss: &Truss, path: &str) -> Result<(), Box<dyn std::error::Error>> {

    let min_x = truss.nodes.iter().map(|n| n.position.x).fold(f64::INFINITY, f64::min);
    let min_y = truss.nodes.iter().map(|n| n.position.y).fold(f64::INFINITY, f64::min);
    let max_x = truss.nodes.iter().map(|n| n.position.x).fold(f64::NEG_INFINITY, f64::max);
    let max_y = truss.nodes.iter().map(|n| n.position.y).fold(f64::NEG_INFINITY, f64::max);

    let width = max_x - min_x;
    let height = max_y - min_y;

    let scale_min_x = min_x - width * 0.1;
    let scale_max_x = max_x + width * 0.1;
    let scale_min_y = min_y - height * 0.1;
    let scale_max_y = max_y + height * 0.1;

    let pixel_heigh = 720;
    let pixel_width = 1280;


    let root = BitMapBackend::new(path, (pixel_width, pixel_heigh)).into_drawing_area();
    root.fill(&WHITE);
    let root = root.margin(10, 10, 10, 10);
    // After this point, we should be able to construct a chart context
    let mut chart = ChartBuilder::on(&root)
        // Set the caption of the chart
        .caption("Undeformed", ("sans-serif", 40).into_font())
        // Set the size of the label region
        .x_label_area_size(20)
        .y_label_area_size(40)
        // Finally attach a coordinate on the drawing area and make a chart context
        .build_cartesian_2d(scale_min_x..scale_max_x, scale_min_y..scale_max_y)?;

    // Then we can draw a mesh
    chart
        .configure_mesh()
        // We can customize the maximum number of labels allowed for each axis
        .x_labels(5)
        .y_labels(5)
        // We can also change the format of the label text
        .y_label_formatter(&|x| format!("{:.3}", x))
        .draw()?;


    // One LineSeries per element, so each line can get its own style later.
    for element in &truss.elements {
        let p1 = (element.node_1.position.x, element.node_1.position.y);
        let p2 = (element.node_2.position.x, element.node_2.position.y);
        chart.draw_series(LineSeries::new(vec![p1, p2], RED.stroke_width(3)))?;
    }

    // plot boundary conditions as black and free dofs as green

    // boundary conditions
    let boundary_points: Vec<(f64, f64)> = truss
        .boundary_conditions
        .iter()
        .map(|bc| (bc.node.position.x, bc.node.position.y))
        .collect();

    chart.draw_series(PointSeries::of_element(
        boundary_points.clone(),
        5,
        &BLACK,
        &|c, s, st| {
            return EmptyElement::at(c)    // We want to construct a composed element on-the-fly
            + Circle::new((0,0),s,st.filled()) // At this point, the new pixel coordinate is established
            + Text::new(format!("{:?}", c), (10, 0), ("sans-serif", 18).into_font());
        },
    ))?;

    // free dofs
    let points: Vec<(f64, f64)> = truss
        .nodes
        .iter()
        .filter(|n| !truss.boundary_conditions.iter().any(|bc| bc.node.id == n.id))
        .map(|n| (n.position.x, n.position.y))
        .collect();

    chart.draw_series(PointSeries::of_element(
        points.clone(),
        5,
        &BLUE,
        &|c, s, st| {
            return EmptyElement::at(c)    // We want to construct a composed element on-the-fly
            + Circle::new((0,0),s,st.filled()) // At this point, the new pixel coordinate is established
            + Text::new(format!("{:?}", c), (10, 0), ("sans-serif", 10).into_font());
        },
    ))?;

    
    root.present()?;
    Ok(())
}