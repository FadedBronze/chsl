use std::{collections::{HashMap, HashSet}, usize};

use crate::math::vector2::Vector2;
use super::bounding_box::{BoundingBox, GetBounds};

#[derive(Debug, Clone, Copy)]
pub struct SpatialInfo {
    inserted_bounds: (usize, usize, usize, usize),
}

pub struct SpatialGrid {
    elements: HashMap<String, SpatialInfo>,
    grid: Vec<HashSet<String>>,
    pub bounds: BoundingBox,
    pub cell_count_x: usize,
    pub cell_count_y: usize,
}

impl SpatialGrid {
    pub fn new(bounds: BoundingBox, cell_count_x: usize, cell_count_y: usize) -> Self {
        let grid = vec![HashSet::new(); cell_count_y * cell_count_x];
        SpatialGrid { bounds, grid, cell_count_x, cell_count_y, elements: HashMap::new() }
    }

    pub fn get_grid_pos(&self, position: Vector2) -> (usize, usize) {
        let cell_size = Vector2::new(self.bounds.width / (self.cell_count_x as f64), self.bounds.height / (self.cell_count_y as f64));

        let index = Vector2::new(
            (position.x - self.bounds.x) / cell_size.x, 
            (position.y - self.bounds.y) / cell_size.y
        );

        return (index.x.floor() as usize, index.y.floor() as usize);
    }

    pub fn get_index(&self, grid_pos: (usize, usize)) -> usize {
        grid_pos.1 * self.cell_count_x + grid_pos.0
    } 

    fn get_grid_bounds<T: GetBounds>(&self, element: &T) -> (usize, usize, usize, usize) {
        let bounds = element.get_bounds();
        let (min_x, min_y) = self.get_grid_pos(Vector2::new(bounds.x, bounds.y));
        let (max_x, max_y) = self.get_grid_pos(Vector2::new(bounds.x + bounds.width, bounds.y + bounds.height)); 
        (min_x.max(0), min_y.max(0), max_x.min(self.cell_count_x-1), max_y.min(self.cell_count_y-1))
    }

    pub fn insert<T: GetBounds>(&mut self, id: &String, element: &mut T) {
        let (min_x, min_y, max_x, max_y) = self.get_grid_bounds(element);

        for y in min_y..=max_y {
            for x in min_x..=max_x {
                let idx = self.get_index((x, y));
                self.grid[idx].insert(id.clone());
            }
        }

        self.elements.insert(id.clone(), SpatialInfo { inserted_bounds: self.get_grid_bounds(element) });
    }

    pub fn remove<T: GetBounds>(&mut self, id: &String) {
        let Some(elem) = self.elements.get(id) else {return;};
        let (min_x, min_y, max_x, max_y) = elem.inserted_bounds;

        self.elements.remove(id).unwrap();

        for y in min_y..=max_y {
            for x in min_x..=max_x {
                let idx = self.get_index((x, y));
                self.grid[idx].remove(id);
            }
        }
    }

    pub fn update<T: GetBounds>(&mut self, id: &String, element: &mut T) {
        self.remove::<T>(id);
        self.insert(id, element);
    }

    pub fn potential_collisions<T: GetBounds>(&mut self, elements: &mut HashMap<String, T>, handle: fn(&mut T, &mut T) -> ()) {
        let mut pot_colls = HashSet::new();

        //let width = self.bounds.width / self.cell_count_x as f64;
        //let height = self.bounds.height / self.cell_count_y as f64;

        for y in 0..self.cell_count_y {
            for x in 0..self.cell_count_x {
                //BoundingBox {x: x as f64 * width, y: y as f64 * height, width, height}.debug_draw(debug_lines);

                //self.bounds.debug_draw(debug_lines);

                let cell_idx = self.get_index((x, y));
                let cell = &mut self.grid[cell_idx];
                let cell_iter: Vec<String> = cell.iter().cloned().collect();

                for i in 0..cell_iter.len() {
                    for j in i+1..cell_iter.len() {
                        //BoundingBox {x: x as f64 * width, y: y as f64 * height, width, height}.debug_x(debug_lines);
                        pot_colls.insert((cell_iter[i].clone(), cell_iter[j].clone()));
                    }
                }
            }
        }

        for (a, b) in pot_colls {
            let mut a_elem = elements.remove(&a).unwrap();
            let mut b_elem = elements.remove(&b).unwrap();

            handle(&mut a_elem, &mut b_elem);

            elements.insert(a, a_elem);
            elements.insert(b, b_elem);
        }
    }
}
