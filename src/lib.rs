use std::collections::hash_map::Entry;
use std::collections::HashMap;
use std::ops::{Index, IndexMut};
use arrayvec::ArrayVec;

pub mod geometry_util;

#[cfg(feature = "algorithms")]
pub mod algorithms;

pub type Hexagonish<T> = ArrayVec<T, 6>;

pub struct Hexasphere<T> {
    subdivisions: usize,
    top: T,
    bottom: T,
    chunks: [Vec<T>; 5],
}

impl<T> Hexasphere<T> {
    /// Assumes that the top index is 0
    /// in the hexasphere geometry crate.
    pub fn from_hexasphere_geometry(subdivisions: usize, indices: &[u32], mut make: impl FnMut(u32, Coordinate) -> T) -> (Self, HashMap<u32, Hexagonish<u32>>) {
        assert_eq!(indices.len() % 3, 0);
        let mut coordinate_store: HashMap<u32, Hexagonish<u32>> = HashMap::new();
        indices
            .chunks(3)
            .for_each(|x| {
                if let &[a, b, c] = x {
                    let mut store_entry = |i, j, k| {
                        match coordinate_store.entry(i) {
                            Entry::Vacant(x) => { x.insert(ArrayVec::from_iter([j, k])); },
                            Entry::Occupied(x) => {
                                let list = x.into_mut();
                                if let Some(idx_j) = list.iter().position(|&z| z == j) {
                                    if list[(idx_j + 1) % list.len()] != k {
                                        list.insert(idx_j + 1, k);
                                    }
                                } else if let Some(idx_k) = list.iter().position(|&z| z == k) {
                                    list.insert(idx_k, j);
                                }
                            }
                        }
                    };

                    // Order of the entries in the arrays matters!
                    store_entry(a, b, c);
                    store_entry(b, c, a);
                    store_entry(c, a, b);
                }
            });

        let top = make(0, Coordinate::Top);
        let bottom = make(11, Coordinate::Bottom);

        let mut chunks = [Vec::new(), Vec::new(), Vec::new(), Vec::new(), Vec::new()];

        let rotate_by = |previous: u32, this: u32, by: isize| {
            let list = coordinate_store.get(&this).unwrap();
            let idx = list.iter().position(|&x| x == previous).unwrap();
            list[(idx as isize + by).rem_euclid(list.len() as isize) as usize]
        };

        for chunk in 0..5 {
            let data = &mut chunks[chunk];
            let mut short_prev = 0;
            let mut short_root = coordinate_store.get(&0).unwrap()[chunk];

            for short in 0..subdivisions + 1 {
                data.push(make(short_root, coord(chunk as u8, short, 0)));

                let mut long_root = rotate_by(short_prev, short_root, -2);
                let mut long_prev = short_root;

                for long in 1..2 * (subdivisions + 1) {
                    data.push(make(long_root, coord(chunk as u8, short, long)));

                    let new_long_root = rotate_by(long_prev, long_root, -3);
                    long_prev = long_root;
                    long_root = new_long_root;
                }

                let new_short_root = rotate_by(short_prev, short_root, -3);
                short_prev = short_root;
                short_root = new_short_root;
            }
        }

        (
            Self {
                subdivisions,
                top,
                bottom,
                chunks
            },
            coordinate_store,
        )
    }

    pub fn surrounding(&self, x: Coordinate) -> Hexagonish<Coordinate> {
        match x {
            Coordinate::Top => [
                coord(0, 0, 0),
                coord(1, 0, 0),
                coord(2, 0, 0),
                coord(3, 0, 0),
                coord(4, 0, 0),
            ].as_ref().try_into().unwrap(),
            Coordinate::Bottom => [
                coord(0, self.subdivisions, 2 * self.subdivisions + 1),
                coord(1, self.subdivisions, 2 * self.subdivisions + 1),
                coord(2, self.subdivisions, 2 * self.subdivisions + 1),
                coord(3, self.subdivisions, 2 * self.subdivisions + 1),
                coord(4, self.subdivisions, 2 * self.subdivisions + 1),
            ].as_ref().try_into().unwrap(),
            Coordinate::Inside {
                chunk,
                short,
                long,
            } => {
                if self.subdivisions == 0 {
                    return if long == 0 {
                        [
                            Coordinate::Top,
                            coord((chunk + 4) % 5, 0, 0),
                            coord((chunk + 4) % 5, 0, 1),
                            coord((chunk + 1) % 5, 0, 0),
                            coord(chunk, 0, 1),
                        ].as_ref().try_into().unwrap()
                    } else {
                        [
                            coord(chunk, 0, 0),
                            coord((chunk + 4) % 5, 0, 1),
                            Coordinate::Bottom,
                            coord((chunk + 1) % 5, 0, 1),
                            coord((chunk + 1) % 5, 0, 0),
                        ].as_ref().try_into().unwrap()
                    };
                }

                if short == self.subdivisions {
                    return if long == 0 {
                        [
                            coord(chunk, short - 1, 0),
                            coord((chunk + 4) % 5, 0, short),
                            coord((chunk + 4) % 5, 0, short + 1),
                            coord(chunk, short, 1),
                            coord(chunk, short - 1, 1),
                        ].as_ref().try_into().unwrap()
                    } else if long == self.subdivisions + 1 {
                        [
                            coord(chunk, short, long - 1),
                            coord((chunk + 4) % 5, 0, short + long),
                            coord(chunk, short, long + 1),
                            coord(chunk, short - 1, long + 1),
                            coord(chunk, short - 1, long),
                        ].as_ref().try_into().unwrap()
                    } else if long <= self.subdivisions {
                        [
                            coord(chunk, short, long - 1),
                            coord((chunk + 4) % 5, 0, short + long),
                            coord((chunk + 4) % 5, 0, short + long + 1),
                            coord(chunk, short, long + 1),
                            coord(chunk, short - 1, long + 1),
                            coord(chunk, short - 1, long),
                        ].into()
                    } else if long <= 2 * self.subdivisions {
                        [
                            coord(chunk, short, long - 1),
                            coord((chunk + 4) % 5, long - self.subdivisions - 2, self.subdivisions * 2 + 1),
                            coord((chunk + 4) % 5, long - self.subdivisions - 1, self.subdivisions * 2 + 1),
                            coord(chunk, short, long + 1),
                            coord(chunk, short - 1, long + 1),
                            coord(chunk, short - 1, long),
                        ].into()
                    } else {
                        [
                            coord(chunk, short, long - 1),
                            coord((chunk + 4) % 5, short - 1, long),
                            coord((chunk + 4) % 5, short, long),
                            Coordinate::Bottom,
                            coord((chunk + 1) % 5, short, long),
                            coord(chunk, short - 1, long),
                        ].into()
                    };
                }

                if short == 0 {
                    return if long == 0 {
                        [
                            Coordinate::Top,
                            coord((chunk + 4) % 5, 0, 0),
                            coord((chunk + 4) % 5, 0, 1),
                            coord(chunk, 1, 0),
                            coord(chunk, 0, 1),
                            coord((chunk + 1) % 5, 0, 0),
                        ].into()
                    } else if long <= self.subdivisions {
                        [
                            coord(chunk, 0, long - 1),
                            coord(chunk, 1, long - 1),
                            coord(chunk, 1, long),
                            coord(chunk, 0, long + 1),
                            coord((chunk + 1) % 5, long, 0),
                            coord((chunk + 1) % 5, long - 1, 0),
                        ].into()
                    } else if long <= 2 * self.subdivisions {
                        [
                            coord(chunk, 0, long - 1),
                            coord(chunk, 1, long - 1),
                            coord(chunk, 1, long),
                            coord(chunk, 0, long + 1),
                            coord((chunk + 1) % 5, self.subdivisions, long - self.subdivisions),
                            coord((chunk + 1) % 5, self.subdivisions, (long - self.subdivisions) - 1),
                        ].into()
                    } else {
                        [
                            coord(chunk, 0, long - 1),
                            coord(chunk, 1, long - 1),
                            coord(chunk, 1, long),
                            coord((chunk + 1) % 5, self.subdivisions, self.subdivisions + 2),
                            coord((chunk + 1) % 5, self.subdivisions, self.subdivisions + 1),
                            coord((chunk + 1) % 5, self.subdivisions, self.subdivisions),
                        ].as_ref().try_into().unwrap()
                    };
                }

                return if long == 0 {
                    [
                        coord(chunk, short - 1, 0),
                        coord((chunk + 4) % 5, 0, short),
                        coord((chunk + 4) % 5, 0, short + 1),
                        coord(chunk, short + 1, 0),
                        coord(chunk, short, 1),
                        coord(chunk, short - 1, 1),
                    ].into()
                } else if long <= 2 * self.subdivisions {
                    [
                        coord(chunk, short, long - 1),
                        coord(chunk, short + 1, long - 1),
                        coord(chunk, short + 1, long),
                        coord(chunk, short, long + 1),
                        coord(chunk, short - 1, long + 1),
                        coord(chunk, short - 1, long),
                    ].into()
                } else {
                    [
                        coord(chunk, short, long - 1),
                        coord(chunk, short + 1, long - 1),
                        coord(chunk, short + 1, long),
                        coord((chunk + 1) % 5, self.subdivisions, self.subdivisions + 2 + short),
                        coord((chunk + 1) % 5, self.subdivisions, self.subdivisions + 1 + short),
                        coord(chunk, short - 1, long),
                    ].into()
                };
            }
        }
    }

    pub fn is_valid(&self, coord: Coordinate) -> bool {
        match coord {
            Coordinate::Top | Coordinate::Bottom => true,
            Coordinate::Inside {
                chunk,
                short,
                long,
            } => chunk < 5 &&
                short <= self.subdivisions &&
                long <= 2 * self.subdivisions + 1
        }
    }

    pub fn subdivisions(&self) -> usize {
        self.subdivisions
    }
}

#[cfg(test)]
impl Hexasphere<()> {
    fn dummy(subdivisions: usize) -> Self{
        Self {
            subdivisions,
            top: (),
            bottom: (),
            chunks: [vec!(), vec!(), vec!(), vec!(), vec!()]
        }
    }
}

impl<T> Index<Coordinate> for Hexasphere<T> {
    type Output = T;

    fn index(&self, index: Coordinate) -> &Self::Output {
        match index {
            Coordinate::Top => &self.top,
            Coordinate::Bottom => &self.bottom,
            Coordinate::Inside {
                chunk,
                short,
                long,
            } => &self.chunks[chunk as usize][short * 2 * (self.subdivisions + 1) + long]
        }
    }
}

impl<T> IndexMut<Coordinate> for Hexasphere<T> {
    fn index_mut(&mut self, index: Coordinate) -> &mut Self::Output {
        match index {
            Coordinate::Top => &mut self.top,
            Coordinate::Bottom => &mut self.bottom,
            Coordinate::Inside {
                chunk,
                short,
                long,
            } => &mut self.chunks[chunk as usize][short * 2 * (self.subdivisions + 1) + long]
        }
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum Coordinate {
    Top,
    Bottom,
    Inside {
        chunk: u8,
        short: usize,
        long: usize,
    }
}

pub fn coord(chunk: u8, short: usize, long: usize) -> Coordinate {
    Coordinate::Inside {
        chunk,
        short,
        long
    }
}

impl Default for Coordinate {
    fn default() -> Self {
        Coordinate::Top
    }
}

#[cfg(test)]
mod tests {
    use std::collections::{HashMap, HashSet};
    use crate::{Coordinate, Hexasphere};

    #[test]
    fn coord_surrounding() {
        let hsphere = Hexasphere::dummy(3);

        let mut to_visit = vec![Coordinate::Top];
        let mut visited = HashMap::<Coordinate, HashSet<Coordinate>>::new();

        while let Some(x) = to_visit.pop() {
            let surrounding = hsphere.surrounding(x);
            let set = surrounding.iter().copied().collect::<HashSet<_>>();

            visited.insert(x, set);
            surrounding.iter().copied().for_each(|z| {
                if !hsphere.is_valid(z) {
                    panic!("Coordinate {:?} yielded {:?} with bad {:?}.", x, surrounding, z);
                }
                if !visited.contains_key(&z) {
                    to_visit.push(z);
                }
            })
        }

        for (x, set) in &visited {
            assert!(!set.contains(x));

            set.iter()
                .for_each(|z| {
                    assert!(visited.get(z).unwrap().contains(x));
                    assert_ne!(z, x);
                });
        }
    }
}
