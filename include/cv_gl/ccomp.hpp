// Copyright Pavlo 2018
#ifndef CV_GL_CCOMP_HPP_
#define CV_GL_CCOMP_HPP_

#include <iostream>
#include <vector>
#include <map>
#include <algorithm>

// std::ostream& operator<<(std::ostream& os, std::pair<int, int>& p) {
//   os << "[" << p.first << ", " << p.second << "]";
//   return os;
// }

template<typename ElemType>
class CComponents {
public:
  CComponents(): count(0) {}

  void Union(ElemType e1, ElemType e2) {
    if (e1 == e2) return;
    int e1_id = GetId(e1);
    int e2_id = GetId(e2);
    // std::cout << "Union: e1 = " << e1 << " (id: " << e1_id 
    //           << "), e2 = " << e2 << " (id: " << e2_id << ")" 
    //           << std::endl;
    int c1 = FindById(e1_id);
    int c2 = FindById(e2_id);
    if (c1 != c2) {
      // connect
      // std::cout << "  connect: " << c1 << ", " << c2 << std::endl;
      if (depth[c1] < depth[c2]) {
        tr[c1] = c2;
        depth[c2] += depth[c1]; //std::max(depth[c1] + 1, depth[c2]);
      } else {
        tr[c2] = c1;
        depth[c1] += depth[c2]; //std::max(depth[c2] + 1, depth[c1]);
      }
      
      --count;
    }
    
  }
  int Find(ElemType e) {
    int id = GetId(e);
    return FindById(id);
  }

  int Connected(ElemType e1, ElemType e2) {
    return (Find(e1) == Find(e2)) ? 1 : 0; 
  }

  std::vector<int> GetComponentIds() const {
    std::vector<int> comp_ids;
    for (int i = 0; i < tr.size(); ++i) {
      if (tr[i] == -1) comp_ids.push_back(i);
    }
    return comp_ids;
  }

  std::vector<ElemType> GetElementsById(int id) {
    std::vector<ElemType> comp_elems;
    for (int i = 0; i < tr.size(); ++i) {
      if (id == FindById(i)) {
        // std::cout << "found: " << i << std::endl;
        comp_elems.push_back(GetEl(i));
      }
    }
    return comp_elems;
  }

  void PrintVec(std::ostream& os = std::cout) const {
    for (int i = 0; i < tr.size(); ++i) {
      os << i << " (" << elems[i] << "): " << tr[i] << ",  depth = " << depth[i] << std::endl;
    }
  }

  int Count() const { return count; }

  template<class Archive>
  void serialize(Archive& archive) {
    archive(elems);
    archive(tr);
    archive(depth);
    archive(ids);
    archive(count);
  }
  
private:
  int GetId(ElemType e) {
    auto it = ids.find(e);
    if (it != ids.end()) {
      return it->second;
    } else  {
      elems.push_back(e);
      tr.push_back(-1);
      depth.push_back(1);
      int e_id = elems.size() - 1;
      ids.emplace(std::make_pair(e, e_id));
      ++count;
      return e_id;
    }
  }
  ElemType GetEl(int id) { return elems[id]; }
  int FindById(int id) {
    while (tr[id] != -1) {
      id = tr[id];
    }
    return id;
  }
  
  // elem to id
  std::vector<ElemType> elems;
  std::vector<int> tr;
  std::vector<int> depth;
  std::map<ElemType, int> ids;
  int count;
};


#endif  // CV_GL_CCOMP_HPP
