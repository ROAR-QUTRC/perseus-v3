export type Direction = "horizontal" | "vertical";
export type Edge = "left" | "right" | "top" | "bottom" | "center";

export interface PanelType {
  type: "panel";
  id: string;
  title: string;
  size: number;
}
export interface PanelGroupType {
  type: "group";
  id: string;
  direction: Direction;
  children: LayoutNodeType[];
  size: number;
}
export type LayoutNodeType = PanelType | PanelGroupType;

// even spacing
const equalize = (group: PanelGroupType) =>
  group.children.forEach((child) => (child.size = 100 / group.children.length));

// recursively (depth first) search the tree for a node with a given id
const findParent = (
  root: LayoutNodeType,
  id: string,
): { parent: PanelGroupType; index: number } | null => {
  if (root.type === "panel") return null;
  for (let i = 0; i < root.children.length; i++) {
    if (root.children[i].id === id) return { parent: root, index: i };
    const hit = findParent(root.children[i], id);
    if (hit) return hit;
  }
  return null;
};

// Collapse single-child groups and merge same-direction nested groups
const normalize = (node: LayoutNodeType): LayoutNodeType => {
  if (node.type === "panel") return node;

  const kids: LayoutNodeType[] = [];
  for (const raw of node.children) {
    const mergedChild = normalize(raw);
    // append children if group of same direction
    if (
      mergedChild.type === "group" &&
      mergedChild.direction === node.direction
    )
      kids.push(...mergedChild.children);
    else kids.push(mergedChild);
  }

  node.children = kids;
  equalize(node);
  // if there is only one child return it otherwise return the node
  if (kids.length === 1) {
    kids[0].size = node.size;
    return kids[0];
  }
  return node;
};

export const movePanel = (
  root: LayoutNodeType,
  sourceId: string,
  targetId: string,
  edge: Edge,
): LayoutNodeType => {
  if (sourceId === targetId) return root;
  const nextTree: LayoutNodeType = JSON.parse(JSON.stringify(root)); // clone $state proxy

  const sourceParent = findParent(nextTree, sourceId);
  let targetParent = findParent(nextTree, targetId);
  if (!sourceParent || !targetParent) return root; // root-only layouts have nothing to move
  const source = sourceParent.parent.children[sourceParent.index] as PanelType;
  const target = targetParent.parent.children[targetParent.index] as PanelType;

  if (edge === "center") {
    // swap widgets between the two panes
    [source.title, target.title] = [target.title, source.title];
    return nextTree;
  }

  // detach source
  sourceParent.parent.children.splice(sourceParent.index, 1);
  equalize(sourceParent.parent);

  // re-find target (indices may have shifted)
  targetParent = findParent(nextTree, targetId)!;
  const dir: Direction =
    edge === "left" || edge === "right" ? "horizontal" : "vertical";
  const after = edge === "right" || edge === "bottom";

  if (targetParent.parent.direction === dir) {
    targetParent.parent.children.splice(
      targetParent.index + (after ? 1 : 0),
      0,
      source,
    );
    equalize(targetParent.parent);
  } else {
    const wrapper: PanelGroupType = {
      type: "group",
      id: crypto.randomUUID(),
      direction: dir,
      size: target.size,
      children: after ? [target, source] : [source, target],
    };
    equalize(wrapper);
    targetParent.parent.children[targetParent.index] = wrapper;
  }
  return normalize(nextTree);
};
