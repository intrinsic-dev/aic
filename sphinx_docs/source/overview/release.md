t# Release Notes

# Theme Style Test: Intrinsic Brand

This page allows you to verify that the **Intrinsic** color palette is rendering correctly across all UI components.

---

## 1. Typography & Links
*Check: Is text `--pst-color-text-base` (Deep Green) and links `--pst-color-link`?*

This is a standard paragraph. It should be high-contrast and easy to read. 
- **[This is a link to Intrinsic](https://intrinsic.ai)** (Check hover color: `--pst-color-link-hover`)
- *This is italicized text.*
- **This is bold text.**
- This is `inline code` (Check: `--pst-color-inline-code`).

---

## 2. Admonitions & Callouts
*Check: Are the backgrounds (`--pst-color-info-bg`) and borders correct?*

:::{note}
**Note Admonition:** This usually uses your primary brand color (`--pst-color-info`).
:::

:::{important}
**Important Admonition:** Check if this stands out from the standard note.
:::

:::{warning}
**Warning Admonition:** Should likely be an amber or gold color (`--pst-color-attention`).
:::

---

## 3. Cards & Surfaces
*Check: Does the card background (`--pst-color-surface`) differ from the page background?*

::::{grid} 2
:gutter: 3

:::{grid-item-card} Intrinsic Deep Green
:shadow: md
**Primary Color:** `#042E27`  
This card tests the surface elevation and border colors.
:::

:::{grid-item-card} Signal Green
:shadow: md
**Accent Color:** `#00FF41`  
Checking how "Bright" set colors look on this surface.
:::
::::

---

## 4. Tables & Lists
*Check: Hover over rows to see `--pst-color-table-row-hover-bg`.*

| Component | Variable Name | Intrinsic Hex |
| :--- | :--- | :--- |
| Sidebar | `--pst-color-surface` | `#F4F7F6` |
| Text | `--pst-color-text-base` | `#042E27` |
| Highlight | `--pst-color-target` | `#00FF41` |

---

## 5. Margin Content
*Check: Does muted text (```--pst-color-text-muted```) appear correctly in the sidebar?*

:::{margin}
**Sidebar Metadata** This text is in the margin. It should be slightly lighter than the main body text.
:::

This paragraph sits next to the margin note. You can check the alignment and the color contrast between the main body and the side content here.

```bash
This is a test
```

```python
int main():
    return true
```

```{note}
This should be a note
```

```{attention}
This should be a note
```

```{caution}
This should be a note
```

```{danger}
This should be a note
```


