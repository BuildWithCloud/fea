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
    root.fill(&WHITE).unwrap();
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
        let node_1 = truss.node(element.node_1_id);
        let node_2 = truss.node(element.node_2_id);
        let p1 = (node_1.position.x, node_1.position.y);
        let p2 = (node_2.position.x, node_2.position.y);
        chart.draw_series(LineSeries::new(vec![p1, p2], RED.stroke_width(3)))?;
    }

    // plot boundary conditions as black and free dofs as green

    // boundary conditions
    let boundary_points: Vec<(f64, f64)> = truss
        .boundary_conditions
        .iter()
        .map(|bc| {
            let node = truss.node(bc.node_id);
            (node.position.x, node.position.y)
        })
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
        .filter(|n| !truss.boundary_conditions.iter().any(|bc| bc.node_id == n.id))
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

pub fn deformed_truss(truss: &Truss, truss_result: &TrussResult, deformation_scale: f64, path: &str) -> Result<(), Box<dyn std::error::Error>>{

    // Deformed position of a node: original position plus scaled displacement.
    let deformed_position = |node: &crate::model::node::Node| -> (f64, f64) {
        truss_result
            .displacements
            .iter()
            .find(|d| d.node_id == node.id)
            .map(|d| {
                (
                    node.position.x + d.x_displacement * deformation_scale,
                    node.position.y + d.y_displacement * deformation_scale,
                )
            })
            .unwrap_or((node.position.x, node.position.y))
    };

    let deformed_nodes: Vec<(f64, f64)> = truss.nodes.iter().map(|n| deformed_position(n)).collect();

    let min_x = deformed_nodes.iter().map(|p| p.0).fold(f64::INFINITY, f64::min);
    let min_y = deformed_nodes.iter().map(|p| p.1).fold(f64::INFINITY, f64::min);
    let max_x = deformed_nodes.iter().map(|p| p.0).fold(f64::NEG_INFINITY, f64::max);
    let max_y = deformed_nodes.iter().map(|p| p.1).fold(f64::NEG_INFINITY, f64::max);

    let width = max_x - min_x;
    let height = max_y - min_y;

    let scale_min_x = min_x - width * 0.1;
    let scale_max_x = max_x + width * 0.1;
    let scale_min_y = min_y - height * 0.1;
    let scale_max_y = max_y + height * 0.1;

    let pixel_heigh = 720;
    let pixel_width = 1280;

    let mut max_stress = f64::NEG_INFINITY;
    let mut min_stress = f64::INFINITY;
    for element in &truss_result.element_results{
        if element.stress > max_stress{
            max_stress = element.stress;
        }
        if element.stress < min_stress{
            min_stress = element.stress;
        }
    }


    let root = BitMapBackend::new(path, (pixel_width, pixel_heigh)).into_drawing_area();
    root.fill(&WHITE).unwrap();
    let root = root.margin(10, 10, 10, 10);
    // After this point, we should be able to construct a chart context
    let mut chart = ChartBuilder::on(&root)
        // Set the caption of the chart
        .caption("Deformed",("sans-serif", 40).into_font())
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
    for element in &truss_result.element_results {
        let p1 = deformed_position(truss.node(element.node_1_id));
        let p2 = deformed_position(truss.node(element.node_2_id));
        let colour = stress_colour(element.stress, min_stress, max_stress);
        chart.draw_series(LineSeries::new(vec![p1, p2], colour.stroke_width(3)))?;
    }

    // plot boundary conditions as black and free dofs as green

    // boundary conditions
    let boundary_points: Vec<(f64, f64)> = truss
        .boundary_conditions
        .iter()
        .map(|bc| deformed_position(truss.node(bc.node_id)))
        .collect();

    chart.draw_series(PointSeries::of_element(
        boundary_points.clone(),
        5,
        &BLACK,
        &|c, s, st| {
            return EmptyElement::at(c)    // We want to construct a composed element on-the-fly
            + Circle::new((0,0),s,st.filled()) // At this point, the new pixel coordinate is established
            + Text::new(format!("({:.2}, {:.2})", c.0, c.1), (10, 0), ("sans-serif", 18).into_font());
        },
    ))?;

    // free dofs
    let points: Vec<(f64, f64)> = truss_result
        .displacements
        .iter()
        .filter(|n| !truss.boundary_conditions.iter().any(|bc| bc.node_id == n.node_id))
        .map(|n| deformed_position(truss.node(n.node_id)))
        .collect();

    chart.draw_series(PointSeries::of_element(
        points.clone(),
        5,
        &BLUE,
        &|c, s, st| {
            return EmptyElement::at(c)    // We want to construct a composed element on-the-fly
            + Circle::new((0,0),s,st.filled()) // At this point, the new pixel coordinate is established
            + Text::new(format!("({:.2}, {:.2})", c.0, c.1), (10, 0), ("sans-serif", 18).into_font());
        },
    ))?;

    // Scale note: a small text box in the top-right corner, drawn in pixel
    // coordinates on the root area so it isn't tied to any data series.
    let (w, _) = root.dim_in_pixel();
    let (box_w, box_h) = (230i32, 30i32);
    let (x0, y0) = (w as i32 - box_w - 10, 10);
    root.draw(&Rectangle::new(
        [(x0, y0), (x0 + box_w, y0 + box_h)],
        ShapeStyle::from(&WHITE.mix(0.8)).filled(),
    ))?;
    root.draw(&Rectangle::new(
        [(x0, y0), (x0 + box_w, y0 + box_h)],
        BLACK.stroke_width(1),
    ))?;
    root.draw(&Text::new(
        format!("Deformation scale: x{:e}", deformation_scale),
        (x0 + 8, y0 + 6),
        ("sans-serif", 18).into_font(),
    ))?;

    root.present()?;
    Ok(())
}

/// Map `value` in [min, max] to a colour from green (min) to red (max) by sweeping hue.
fn stress_colour(value: f64, min: f64, max: f64) -> HSLColor {
    let range = max - min;
    // Guard against divide-by-zero when all stresses are equal.
    let t = if range.abs() < f64::EPSILON { 0.0 } else { ((value - min) / range).clamp(0.0, 1.0) };
    // Hue 1/3 = green, 0 = red.
    HSLColor(1.0 / 3.0 * (1.0 - t), 1.0, 0.45)
}