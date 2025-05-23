// ********************************************** open the readme files for different contents *************************************************************/

async function loadMarkdown(directory, fileName, id) {
  try {
    // Fetch the Markdown content from the server
    const response = await fetch(
      `/get-markdown?directory=${encodeURIComponent(
        directory
      )}&file=${encodeURIComponent(fileName)}`
    );
    const data = await response.json();

    const contentElement = document.getElementById(id);

    if (data.error) {
      contentElement.textContent = data.error;
    } else {
      const markdownContent = data.markdown;
      const htmlContent = marked.parse(markdownContent);
      contentElement.innerHTML = htmlContent;
      contentElement.classList.add("active");
      contentElement.classList.remove("hidden");
    }
  } catch (error) {
    console.error("Error fetching Markdown:", error);
    document.getElementById("markdownContent").textContent =
      "Failed to load content.";
  }
}

// open the documents
document.addEventListener("DOMContentLoaded", function () {
  const links = document.querySelectorAll(".main_menu a");
  const sections = document.querySelectorAll(".markdown-body");

  links.forEach((link) => {
    link.addEventListener("click", function (event) {
      event.preventDefault(); // Prevent default anchor behavior

      // Hide all sections
      sections.forEach((section) => {
        section.classList.remove("active");
        section.classList.add("hidden");
      });

      // Show the clicked section
      const targetId = this.getAttribute("href").substring(1);
      const targetSection = document.getElementById(targetId);

      if (targetId === "evalkit-readme-content") {
        loadMarkdown(
          "web-1.0.0/static/docs",
          "evalkit_readme.md",
          "evalkit-readme-content"
        );
      } else if (targetId === "webinterface") {
        loadMarkdown(
          "web-1.0.0/static/docs",
          "webinterface.md",
          "webinterface"
        );
      } else {
        targetSection.classList.add("active");
        targetSection.classList.remove("hidden");
      }
    });
  });
});
